#include "Actors/SolarUAV.h"
#include "Components/StaticMeshComponent.h"
#include "UObject/ConstructorHelpers.h"
#include "DrawDebugHelpers.h"
#include "EnhancedInputComponent.h"
#include "InputActionValue.h"

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

void ASolarUAV::FixedPhysicsTick(float FixedDt)
{
	FVector UAVWorldPos = RigidBody.GetWorldPosition();

	FVector WindAtUAV = WindField.SampleWindVelocity(UAVWorldPos);
	float   AirDensity = WindField.SampleAirDensity(UAVWorldPos);

	FVector UAVWorldVel = RigidBody.GetLinearVelocity();
	FVector RelativeAirspeed = UAVWorldVel - WindAtUAV;

	FQuat BodyOrientation = RigidBody.GetWorldOrientation();
	FVector AngVelocity = RigidBody.GetAngularVelocity();

	LastAeroResult = AeroSolver.ComputeAerodynamicForces(
		RelativeAirspeed,
		AngVelocity,
		BodyOrientation,
		AirDensity,
		FixedDt
	);

	RigidBody.AddForce_World(LastAeroResult.TotalForce_World);
	RigidBody.AddTorque_World(LastAeroResult.TotalTorque_World);

	ApplyControlSurfaceForces();

	FVector Gravity(0.0f, 0.0f, -GravityMagnitude);
	RigidBody.Integrate(FixedDt, Gravity);

	SimElapsed += FixedDt;
}

void ASolarUAV::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	if (!bInitialized) return;

	WindField.UpdateField(SimElapsed, DeltaSeconds);

	PhysicsAccumulator += DeltaSeconds;

	int32 MaxSubsteps = FMath::CeilToInt(DeltaSeconds / FixedPhysicsStep);
	MaxSubsteps = FMath::Min(MaxSubsteps, 50);

	int32 Steps = 0;
	while (PhysicsAccumulator >= FixedPhysicsStep && Steps < MaxSubsteps)
	{
		FixedPhysicsTick(FixedPhysicsStep);
		PhysicsAccumulator -= FixedPhysicsStep;
		Steps++;
	}

	if (PhysicsAccumulator > FixedPhysicsStep * 2.0f)
	{
		PhysicsAccumulator = 0.0f;
	}

	FVector NewPos = RigidBody.GetWorldPosition();
	FQuat   NewQuat = RigidBody.GetWorldOrientation();

	SetActorLocationAndRotation(NewPos, NewQuat);

	UpdateTelemetry();

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
	CurrentMicroburstIntensity = WindField.GetMicroburstIntensityAtTime(SimElapsed);
	CurrentFlutterAmplitude = AeroSolver.GetFlutterAmplitude();
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
	SimElapsed = 0.0f;
	PhysicsAccumulator = 0.0f;
	SetActorLocationAndRotation(Position, Orientation);
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
		// Enhanced Input bindings would be set here via Input Actions/Contexts
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
