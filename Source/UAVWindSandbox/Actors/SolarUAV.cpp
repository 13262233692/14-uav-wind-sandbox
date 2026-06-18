#include "Actors/SolarUAV.h"
#include "Components/StaticMeshComponent.h"
#include "UObject/ConstructorHelpers.h"
#include "DrawDebugHelpers.h"
#include "EnhancedInputComponent.h"
#include "InputActionValue.h"
#include "Engine/World.h"
#include "Engine/Engine.h"

ASolarUAV::ASolarUAV()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickGroup = TG_PostPhysics;

	bUseControllerRotationPitch = false;
	bUseControllerRotationYaw   = false;
	bUseControllerRotationRoll  = false;

	USceneComponent* Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	RootComponent = Root;

	AircraftConfig.WingSpanM = 25.0f;
	AircraftConfig.WingAreaM2 = 12.0f;
	AircraftConfig.AspectRatio = (AircraftConfig.WingSpanM * AircraftConfig.WingSpanM) / AircraftConfig.WingAreaM2;
	AircraftConfig.MassKg = 50.0f;
	AircraftConfig.InertiaDiagonal = FVector(85.0f, 8.0f, 90.0f);
	AircraftConfig.OswaldEfficiency = 0.92f;
	AircraftConfig.MeanAerodynamicChord = 0.48f;
	AircraftConfig.ClAlphaPerRad = 5.8f;
	AircraftConfig.CdMin = 0.015f;
	AircraftConfig.StallAlphaDeg = 16.0f;
	AircraftConfig.FlutterFrequencyHz = 3.5f;
	AircraftConfig.FlutterDampingRatio = 0.15f;
	AircraftConfig.FlexuralRigidity = 2500.0f;
	AircraftConfig.TorsionalRigidity = 180.0f;
}

void ASolarUAV::BeginPlay()
{
	Super::BeginPlay();
	InitializePhysics();
}

void ASolarUAV::InitializePhysics()
{
	AeroSolver.Initialize(AircraftConfig);

	RigidBody.SetMass(AircraftConfig.MassKg);
	RigidBody.SetInertiaTensor(AircraftConfig.InertiaDiagonal);

	FVector StartPos = GetActorLocation();
	FQuat   StartQuat = GetActorQuat();
	RigidBody.SetInitialState(StartPos, StartQuat);

	WindField.Initialize(WindFieldBounds, WindFieldCellSize);
	WindField.SetMicroburstConfig(MicroburstConfig);
	WindField.SetAmbientWind(AmbientWindVelocity);

	SimElapsed = 0.0f;
	PhysicsAccumulator = 0.0f;
	bInitialized = true;
	bEmergencyStopped = false;
	SystemHealth = EUAVSystemHealth::Healthy;
	ConsecutiveRecoveries = 0;
	TotalEmergencyRecoveries = 0;
	TotalAeroEmergencyClamps = 0;
	LastHealthMessage = TEXT("System initialized");
}

void ASolarUAV::SetControlSurfaces(float Aileron, float Elevator, float Rudder, float Thr)
{
	AileronInput  = FMath::Clamp(Aileron, -1.0f, 1.0f);
	ElevatorInput = FMath::Clamp(Elevator, -1.0f, 1.0f);
	RudderInput   = FMath::Clamp(Rudder, -1.0f, 1.0f);
	ThrottleInput = FMath::Clamp(Thr, 0.0f, 1.0f);
}

void ASolarUAV::ApplyControlSurfaceForces()
{
	FQuat BodyQuat = RigidBody.GetWorldOrientation();

	FVector AileronTorque_Body(
		AileronInput * MaxAileronMoment,
		0.0f,
		0.0f
	);
	FVector ElevatorTorque_Body(
		0.0f,
		ElevatorInput * MaxElevatorMoment,
		0.0f
	);
	FVector RudderTorque_Body(
		0.0f,
		0.0f,
		RudderInput * MaxRudderMoment
	);

	FVector TotalTorque_Body = AileronTorque_Body + ElevatorTorque_Body + RudderTorque_Body;
	FVector TotalTorque_World = BodyQuat.RotateVector(TotalTorque_Body);

	RigidBody.AddTorque_World(TotalTorque_World);

	FVector ThrustDir_Body(1.0f, 0.0f, 0.0f);
	FVector ThrustDir_World = BodyQuat.RotateVector(ThrustDir_Body);
	float ThrustMag = ThrottleInput * MaxThrustForce;

	RigidBody.AddForce_World(ThrustDir_World * ThrustMag);
}

void ASolarUAV::ComputeAndApplyAerodynamics(float StepTime)
{
	FVector UAVWorldPos = RigidBody.GetWorldPosition();

	FVector WindAtUAV = WindField.SampleWindVelocity(UAVWorldPos);
	float   AirDensity = WindField.SampleAirDensity(UAVWorldPos);

	FVector UAVWorldVel = RigidBody.GetLinearVelocity();
	FVector RelativeAirspeed = UAVWorldVel - WindAtUAV;

	FQuat BodyOrientation = RigidBody.GetWorldOrientation();
	FVector AngVelocity = RigidBody.GetAngularVelocity();

	FAeroForceResult AeroResult = AeroSolver.ComputeAerodynamicForces(
		RelativeAirspeed,
		AngVelocity,
		BodyOrientation,
		AirDensity,
		StepTime
	);

	RigidBody.AddForce_World(AeroResult.TotalForce_World);
	RigidBody.AddTorque_World(AeroResult.TotalTorque_World);

	LastAeroResult = AeroResult;
}

void ASolarUAV::FixedPhysicsTick_LegacyEuler(float FixedDt)
{
	ComputeAndApplyAerodynamics(FixedDt);
	ApplyControlSurfaceForces();

	FVector Gravity(0.0f, 0.0f, -GravityMagnitude);

	FRigidBodyDerivative6DOF k1 = RigidBody.ComputeDerivatives(RigidBody.GetState(), Gravity);
	FRigidBodyState6DOF NewState = RigidBody.AdvanceState(RigidBody.GetState(), k1, FixedDt);
	RigidBody.SetState(NewState);

	SimElapsed += FixedDt;
}

void ASolarUAV::FixedPhysicsTick_RK4(float FixedDt)
{
	ComputeAndApplyAerodynamics(FixedDt);
	ApplyControlSurfaceForces();

	FVector Gravity(0.0f, 0.0f, -GravityMagnitude);
	RigidBody.Integrate_RK4(FixedDt, Gravity);

	SimElapsed += FixedDt;
}

void ASolarUAV::FixedPhysicsTick_AdaptiveRK4(float FrameDt)
{
	FVector Gravity(0.0f, 0.0f, -GravityMagnitude);

	float TimeDilation = GetWorld() ? GetWorld()->GetWorldSettings()->GetEffectiveTimeDilation() : 1.0f;
	float ShearScale = 1.0f;

	if (bAutoReduceStepAtHighWindShear && CurrentWindShearMagnitude > WindShearStepReductionThreshold)
	{
		ShearScale = FMath::Lerp(1.0f, 0.2f,
			(CurrentWindShearMagnitude - WindShearStepReductionThreshold) / 30.0f);
		ShearScale = FMath::Clamp(ShearScale, 0.2f, 1.0f);
	}

	float AdaptiveMaxStep = UAV_MAX_STEP_TIME * ShearScale;

	ComputeAndApplyAerodynamics(FrameDt * 0.5f);
	ApplyControlSurfaceForces();

	float StepTime = FrameDt;
	RigidBody.Integrate_AdaptiveRK4(StepTime, Gravity);

	SimElapsed += StepTime;
}

void ASolarUAV::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	if (!bInitialized || bEmergencyStopped) return;

	EngineTimeDilation = GetWorld() ? GetWorld()->GetWorldSettings()->GetEffectiveTimeDilation() : 1.0f;

	WindField.UpdateField(SimElapsed, DeltaSeconds);

	FVector UAVWorldPos = RigidBody.GetWorldPosition();
	CurrentWindVelocity  = WindField.SampleWindVelocity(UAVWorldPos);
	CurrentWindShear     = WindField.SampleWindShear(UAVWorldPos);
	CurrentWindShearMagnitude = CurrentWindShear.Size();

	SubStepsThisFrame = 0;
	RejectedStepsThisFrame = 0;

	if (IntegratorMode == EUAVIntegratorMode::RK4_Adaptive)
	{
		FixedPhysicsTick_AdaptiveRK4(DeltaSeconds);
	}
	else if (IntegratorMode == EUAVIntegratorMode::RK4_FixedStep)
	{
		PhysicsAccumulator += DeltaSeconds;

		int32 MaxSubsteps = FMath::CeilToInt(DeltaSeconds / FixedPhysicsStep);
		MaxSubsteps = FMath::Min(MaxSubsteps, 200);

		int32 Steps = 0;
		while (PhysicsAccumulator >= FixedPhysicsStep && Steps < MaxSubsteps)
		{
			FixedPhysicsTick_RK4(FixedPhysicsStep);
			PhysicsAccumulator -= FixedPhysicsStep;
			Steps++;
			SubStepsThisFrame = Steps;
		}
	}
	else
	{
		PhysicsAccumulator += DeltaSeconds;

		int32 MaxSubsteps = FMath::CeilToInt(DeltaSeconds / FixedPhysicsStep);
		MaxSubsteps = FMath::Min(MaxSubsteps, 200);

		int32 Steps = 0;
		while (PhysicsAccumulator >= FixedPhysicsStep && Steps < MaxSubsteps)
		{
			FixedPhysicsTick_LegacyEuler(FixedPhysicsStep);
			PhysicsAccumulator -= FixedPhysicsStep;
			Steps++;
			SubStepsThisFrame = Steps;
		}
	}

	if (PhysicsAccumulator > FixedPhysicsStep * 2.0f)
	{
		PhysicsAccumulator = 0.0f;
	}

	FIntegrationStats Stats = RigidBody.GetIntegrationStats();
	SubStepsThisFrame = FMath::Max(SubStepsThisFrame, Stats.SubStepsTaken);
	RejectedStepsThisFrame = Stats.RejectedSteps;
	LastTruncationError = Stats.LastErrorEstimate;
	CurrentAdaptiveStepSize = Stats.CurrentStepSize;
	EffectivePhysicsStep = CurrentAdaptiveStepSize;

	FVector NewPos = RigidBody.GetWorldPosition();
	FQuat   NewQuat = RigidBody.GetWorldOrientation();

	SetActorLocationAndRotation(NewPos, NewQuat);

	UpdateTelemetry();
	UpdateSystemHealth();

	if (bDrawDebugForces) DrawDebugVisualization();
	if (bDrawDebugWind) DrawWindFieldVisualization();
}

void ASolarUAV::UpdateTelemetry()
{
	FVector UAVWorldVel = RigidBody.GetLinearVelocity();
	FVector WindAtUAV = WindField.SampleWindVelocity(RigidBody.GetWorldPosition());

	CurrentWorldVelocity = UAVWorldVel;
	CurrentWindVelocity  = WindAtUAV;
	CurrentAirspeedVec   = UAVWorldVel - WindAtUAV;
	CurrentAirspeed      = CurrentAirspeedVec.Size();
	CurrentAlphaDeg      = AeroSolver.GetAlphaDeg();
	CurrentBetaDeg       = AeroSolver.GetBetaDeg();
	CurrentDynamicPressure = AeroSolver.GetDynamicPressure();
	CurrentCL            = AeroSolver.GetLiftCoefficient();
	CurrentCD            = AeroSolver.GetDragCoefficient();
	CurrentWindShear     = WindField.SampleWindShear(RigidBody.GetWorldPosition());
	CurrentWindShearMagnitude = CurrentWindShear.Size();
	CurrentMicroburstIntensity = WindField.GetMicroburstIntensityAtTime(SimElapsed);
	CurrentFlutterAmplitude = AeroSolver.GetFlutterAmplitude();

	TotalEmergencyRecoveries = RigidBody.GetTotalEmergencyRecoveries();
	TotalAeroEmergencyClamps = AeroSolver.GetTotalEmergencyClamps();
}

void ASolarUAV::UpdateSystemHealth()
{
	EIntegratorHealth BodyHealth = RigidBody.GetHealthStatus();
	EAeroSolverHealth AeroHealth = AeroSolver.GetHealth();

	EUAVSystemHealth NewHealth = EUAVSystemHealth::Healthy;

	if (BodyHealth == EIntegratorHealth::EmergencyStop ||
		AeroHealth == EAeroSolverHealth::EmergencyClamped)
	{
		NewHealth = EUAVSystemHealth::EmergencyStop;
		LastHealthMessage = TEXT("EMERGENCY STOP: Numerical instability detected");
	}
	else if (BodyHealth == EIntegratorHealth::StateRecovered)
	{
		NewHealth = EUAVSystemHealth::StateRecovery;
		ConsecutiveRecoveries++;
		LastHealthMessage = FString::Printf(TEXT("STATE RECOVERY #%d: Rolled back to last valid state"),
			TotalEmergencyRecoveries);
	}
	else if (AeroHealth >= EAeroSolverHealth::OutputClamped)
	{
		NewHealth = EUAVSystemHealth::NumericClamping;
		LastHealthMessage = TEXT("NUMERIC CLAMPING: Aerodynamic values exceeded safe bounds");
	}
	else if (BodyHealth == EIntegratorHealth::Warning || AeroHealth >= EAeroSolverHealth::InputClamped)
	{
		NewHealth = EUAVSystemHealth::NumericClamping;
		LastHealthMessage = TEXT("NUMERIC CLAMPING: Input values were clamped");
	}
	else
	{
		NewHealth = EUAVSystemHealth::Healthy;
		if (ConsecutiveRecoveries > 0)
		{
			ConsecutiveRecoveries = 0;
		}
		if (SystemHealth != EUAVSystemHealth::Healthy)
		{
			LastHealthMessage = TEXT("HEALTHY: System recovered to nominal operation");
		}
	}

	if (bEnableEmergencyStop && ConsecutiveRecoveries >= MaxConsecutiveRecoveries)
	{
		NewHealth = EUAVSystemHealth::EmergencyStop;
		bEmergencyStopped = true;
		LastHealthMessage = FString::Printf(TEXT("EMERGENCY STOP: %d consecutive state recoveries exceeded limit %d"),
			ConsecutiveRecoveries, MaxConsecutiveRecoveries);
	}

	if (CurrentAirspeed > MaxSafeAirspeed && NewHealth < EUAVSystemHealth::SystemUnstable)
	{
		NewHealth = EUAVSystemHealth::SystemUnstable;
		LastHealthMessage = FString::Printf(TEXT("UNSTABLE: Airspeed %.1f m/s exceeds max safe %.1f m/s"),
			CurrentAirspeed, MaxSafeAirspeed);
	}

	SystemHealth = NewHealth;

	if (GEngine && SystemHealth >= EUAVSystemHealth::NumericClamping)
	{
		FColor MsgColor = FColor::Green;
		if (SystemHealth == EUAVSystemHealth::StateRecovery) MsgColor = FColor::Yellow;
		else if (SystemHealth >= EUAVSystemHealth::SystemUnstable) MsgColor = FColor::Red;

		GEngine->AddOnScreenDebugMessage(-1, 0.1f, MsgColor,
			FString::Printf(TEXT("[UAV HEALTH] %s"), *LastHealthMessage));
	}
}

void ASolarUAV::DrawDebugVisualization()
{
	FVector Pos = RigidBody.GetWorldPosition();
	FQuat Quat = RigidBody.GetWorldOrientation();
	float Scale = DebugDrawScale;

	FVector Forward = Quat.RotateVector(FVector::ForwardVector);
	FVector Right   = Quat.RotateVector(FVector::RightVector);
	FVector Up      = Quat.RotateVector(FVector::UpVector);

	DrawDebugLine(GetWorld(), Pos, Pos + Forward * 100.0f, FColor::Red, false, 0.0f, 0, 2.0f);
	DrawDebugLine(GetWorld(), Pos, Pos + Right * 80.0f,   FColor::Green, false, 0.0f, 0, 2.0f);
	DrawDebugLine(GetWorld(), Pos, Pos + Up * 80.0f,      FColor::Blue, false, 0.0f, 0, 2.0f);

	if (!LastAeroResult.Lift_World.IsNearlyZero())
	{
		DrawDebugLine(GetWorld(), Pos, Pos + LastAeroResult.Lift_World * Scale,
			FColor::Cyan, false, 0.0f, 0, 3.0f);
	}
	if (!LastAeroResult.Drag_World.IsNearlyZero())
	{
		DrawDebugLine(GetWorld(), Pos, Pos + LastAeroResult.Drag_World * Scale,
			FColor::Yellow, false, 0.0f, 0, 3.0f);
	}
	if (!LastAeroResult.FlutterTorque.IsNearlyZero())
	{
		DrawDebugLine(GetWorld(), Pos, Pos + LastAeroResult.FlutterTorque * Scale * 10.0f,
			FColor::Magenta, false, 0.0f, 0, 2.0f);
	}

	FVector AirspeedDir = CurrentAirspeedVec.GetSafeNormal();
	if (CurrentAirspeed > 1.0f)
	{
		DrawDebugLine(GetWorld(), Pos, Pos + AirspeedDir * 60.0f,
			FColor::Orange, false, 0.0f, 0, 2.0f);
	}

	if (SystemHealth >= EUAVSystemHealth::StateRecovery)
	{
		DrawDebugSphere(GetWorld(), Pos, 30.0f, 12,
			SystemHealth >= EUAVSystemHealth::EmergencyStop ? FColor::Red : FColor::Yellow,
			false, 0.0f, 0, 3.0f);
	}

	if (bDrawDebugSubstepMarkers && SubStepsThisFrame > 5)
	{
		FColor SubstepColor = SubStepsThisFrame > 50 ? FColor::Red :
			SubStepsThisFrame > 20 ? FColor::Yellow : FColor::Green;
		DrawDebugString(GetWorld(), Pos + FVector(0, 0, 50),
			FString::Printf(TEXT("Substeps: %d | Rejected: %d | Err: %.2e"),
				SubStepsThisFrame, RejectedStepsThisFrame, LastTruncationError),
			nullptr, SubstepColor, 0.0f, true, 1.0f);
	}
}

void ASolarUAV::DrawWindFieldVisualization()
{
	FVector UAVPos = RigidBody.GetWorldPosition();
	float SampleSpacing = 200.0f;
	float ArrowScale = DebugWindArrowScale;

	int32 Range = 3;
	for (int32 DX = -Range; DX <= Range; DX++)
	{
		for (int32 DY = -Range; DY <= Range; DY++)
		{
			for (int32 DZ = -1; DZ <= 1; DZ++)
			{
				FVector SamplePos = UAVPos + FVector(
					(float)DX * SampleSpacing,
					(float)DY * SampleSpacing,
					(float)DZ * SampleSpacing
				);

				FVector Wind = WindField.SampleWindVelocity(SamplePos);
				float WindMag = Wind.Size();

				if (WindMag < 0.5f) continue;

				FColor ArrowColor;
				if (Wind.Z < -5.0f)
					ArrowColor = FColor::Red;
				else if (WindMag > 15.0f)
					ArrowColor = FColor::Yellow;
				else
					ArrowColor = FColor::Green;

				FVector ArrowEnd = SamplePos + Wind.GetSafeNormal() * FMath::Min(WindMag * ArrowScale, 200.0f);
				DrawDebugLine(GetWorld(), SamplePos, ArrowEnd, ArrowColor, false, 0.0f, 0, 1.5f);

				DrawDebugPoint(GetWorld(), ArrowEnd, 4.0f, ArrowColor, false, 0.0f, 0);
			}
		}
	}
}

void ASolarUAV::ResetToInitial(FVector Position, FRotator Orientation)
{
	RigidBody.SetInitialState(Position, Orientation.Quaternion());
	AeroSolver.Initialize(AircraftConfig);
	SimElapsed = 0.0f;
	PhysicsAccumulator = 0.0f;
	bEmergencyStopped = false;
	ConsecutiveRecoveries = 0;
	SystemHealth = EUAVSystemHealth::Healthy;
	LastHealthMessage = TEXT("System reset");
	SetActorLocationAndRotation(Position, Orientation);
}

void ASolarUAV::ResetSystemHealth()
{
	SystemHealth = EUAVSystemHealth::Healthy;
	ConsecutiveRecoveries = 0;
	bEmergencyStopped = false;
	AeroSolver.ResetHealth();
	RigidBody.ResetIntegrationStats();
	LastHealthMessage = TEXT("Health counters reset");
}

void ASolarUAV::EmergencyStop()
{
	bEmergencyStopped = true;
	SystemHealth = EUAVSystemHealth::EmergencyStop;
	LastHealthMessage = TEXT("MANUAL EMERGENCY STOP");
	if (GEngine)
	{
		GEngine->AddOnScreenDebugMessage(-1, 5.0f, FColor::Red,
			TEXT("[UAV] EMERGENCY STOP ACTIVATED"));
	}
}

void ASolarUAV::TriggerMicroburst(FVector Center, float Intensity)
{
	FMicroburstConfig NewCfg = MicroburstConfig;
	NewCfg.CenterPosition = Center;
	NewCfg.ShearIntensity = Intensity;
	NewCfg.StartTime = SimElapsed;
	NewCfg.PeakTime = SimElapsed + 50.0f;
	WindField.SetMicroburstConfig(NewCfg);
}

FVector ASolarUAV::GetWindAtPosition(FVector Pos) const
{
	return WindField.SampleWindVelocity(Pos);
}

float ASolarUAV::GetAirDensityAtPosition(FVector Pos) const
{
	return WindField.SampleAirDensity(Pos);
}

void ASolarUAV::SetupPlayerInputComponent(UInputComponent* Pic)
{
	Super::SetupPlayerInputComponent(Pic);

	UEnhancedInputComponent* Eic = Cast<UEnhancedInputComponent>(Pic);
	if (Eic)
	{
	}
}

void ASolarUAV::InputThrottle(const FInputActionValue& Value)
{
	ThrottleInput = FMath::Clamp(Value.Get<float>(), 0.0f, 1.0f);
}

void ASolarUAV::InputPitch(const FInputActionValue& Value)
{
	ElevatorInput = FMath::Clamp(Value.Get<float>(), -1.0f, 1.0f);
}

void ASolarUAV::InputRoll(const FInputActionValue& Value)
{
	AileronInput = FMath::Clamp(Value.Get<float>(), -1.0f, 1.0f);
}

void ASolarUAV::InputYaw(const FInputActionValue& Value)
{
	RudderInput = FMath::Clamp(Value.Get<float>(), -1.0f, 1.0f);
}
