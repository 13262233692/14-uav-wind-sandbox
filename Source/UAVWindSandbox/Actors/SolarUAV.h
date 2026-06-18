#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "Physics/RigidBody6DOF.h"
#include "Physics/AerodynamicsSolver.h"
#include "Physics/MicroburstWindField.h"
#include "SolarUAV.generated.h"

UENUM(BlueprintType)
enum class EUAVIntegratorMode : uint8
{
	RK4_FixedStep      UMETA(DisplayName = "RK4 Fixed Step"),
	RK4_Adaptive       UMETA(DisplayName = "RK4 Adaptive Substepping"),
	Euler_Explicit     UMETA(DisplayName = "Legacy Explicit Euler (Not Safe)")
};

UENUM(BlueprintType)
enum class EUAVSystemHealth : uint8
{
	Healthy             UMETA(DisplayName = "Fully Operational"),
	NumericClamping     UMETA(DisplayName = "Numeric Clamping Active"),
	StateRecovery       UMETA(DisplayName = "State Recovery Triggered"),
	SystemUnstable      UMETA(DisplayName = "System Unstable"),
	EmergencyStop       UMETA(DisplayName = "Emergency Stop")
};

UCLASS(Config=Game, BlueprintType, Blueprintable)
class UAVWINDSANDBOX_API ASolarUAV : public APawn
{
	GENERATED_BODY()

public:
	ASolarUAV();

	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;
	virtual void SetupPlayerInputComponent(UInputComponent* Pic) override;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="UAV|Physics")
	EUAVIntegratorMode IntegratorMode = EUAVIntegratorMode::RK4_Adaptive;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="UAV|Physics",
		meta = (ClampMin = "0.00025", ClampMax = "0.01", UIMin = "0.00025", UIMax = "0.01"))
	float FixedPhysicsStep = 0.002f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="UAV|Physics")
	float GravityMagnitude = 9.81f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="UAV|Physics")
	bool bEnableAdaptiveStepWithTimeDilation = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="UAV|Physics")
	bool bAutoReduceStepAtHighWindShear = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="UAV|Physics")
	float WindShearStepReductionThreshold = 15.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="UAV|Aircraft")
	FAircraftConfig AircraftConfig;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="UAV|WindField")
	FMicroburstConfig MicroburstConfig;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="UAV|WindField")
	FVector AmbientWindVelocity = FVector(5.0f, 0.0f, 0.0f);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="UAV|WindField")
	FBox WindFieldBounds = FBox(FVector(-2000, -2000, 0), FVector(2000, 2000, 1500));

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="UAV|WindField")
	float WindFieldCellSize = 50.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="UAV|Debug")
	bool bDrawDebugForces = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="UAV|Debug")
	bool bDrawDebugWind = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="UAV|Debug")
	bool bDrawDebugSubstepMarkers = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="UAV|Debug")
	float DebugDrawScale = 0.02f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="UAV|Debug")
	float DebugWindArrowScale = 3.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="UAV|Safety")
	bool bEnableEmergencyStop = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="UAV|Safety",
		meta = (ClampMin = "1", ClampMax = "100"))
	int32 MaxConsecutiveRecoveries = 10;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="UAV|Safety")
	float MinSafeAirspeed = 10.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="UAV|Safety")
	float MaxSafeAirspeed = 150.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="UAV|Telemetry|Aerodynamics")
	FVector CurrentWorldVelocity;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="UAV|Telemetry|Aerodynamics")
	FVector CurrentAirspeedVec;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="UAV|Telemetry|Aerodynamics")
	float CurrentAirspeed;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="UAV|Telemetry|Aerodynamics")
	float CurrentAlphaDeg;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="UAV|Telemetry|Aerodynamics")
	float CurrentBetaDeg;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="UAV|Telemetry|Aerodynamics")
	float CurrentDynamicPressure;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="UAV|Telemetry|Aerodynamics")
	float CurrentCL;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="UAV|Telemetry|Aerodynamics")
	float CurrentCD;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="UAV|Telemetry|Wind")
	FVector CurrentWindVelocity;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="UAV|Telemetry|Wind")
	FVector CurrentWindShear;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="UAV|Telemetry|Wind")
	float CurrentWindShearMagnitude;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="UAV|Telemetry|Wind")
	float CurrentMicroburstIntensity;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="UAV|Telemetry|Structural")
	float CurrentFlutterAmplitude;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="UAV|Telemetry|Time")
	float SimElapsed;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="UAV|Telemetry|Time")
	float EngineTimeDilation;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="UAV|Telemetry|Time")
	float EffectivePhysicsStep;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="UAV|Telemetry|Integrator")
	int32 SubStepsThisFrame;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="UAV|Telemetry|Integrator")
	int32 RejectedStepsThisFrame;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="UAV|Telemetry|Integrator")
	float LastTruncationError;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="UAV|Telemetry|Integrator")
	float CurrentAdaptiveStepSize;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="UAV|Telemetry|Health")
	EUAVSystemHealth SystemHealth = EUAVSystemHealth::Healthy;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="UAV|Telemetry|Health")
	int32 TotalEmergencyRecoveries;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="UAV|Telemetry|Health")
	int32 TotalAeroEmergencyClamps;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="UAV|Telemetry|Health")
	int32 ConsecutiveRecoveries;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="UAV|Telemetry|Health")
	FString LastHealthMessage;

	UFUNCTION(BlueprintCallable, Category="UAV|Control")
	void SetControlSurfaces(float Aileron, float Elevator, float Rudder, float Throttle);

	UFUNCTION(BlueprintCallable, Category="UAV|Control")
	void ResetToInitial(FVector Position, FRotator Orientation);

	UFUNCTION(BlueprintCallable, Category="UAV|WindField")
	void TriggerMicroburst(FVector Center, float Intensity);

	UFUNCTION(BlueprintCallable, Category="UAV|Safety")
	void ResetSystemHealth();

	UFUNCTION(BlueprintCallable, Category="UAV|Safety")
	void EmergencyStop();

	UFUNCTION(BlueprintPure, Category="UAV|Telemetry")
	FVector GetLiftForce() const { return LastAeroResult.Lift_World; }

	UFUNCTION(BlueprintPure, Category="UAV|Telemetry")
	FVector GetDragForce() const { return LastAeroResult.Drag_World; }

	UFUNCTION(BlueprintPure, Category="UAV|Telemetry")
	FVector GetFlutterTorque() const { return LastAeroResult.FlutterTorque; }

	UFUNCTION(BlueprintPure, Category="UAV|Telemetry")
	FVector GetWindAtPosition(FVector Pos) const;

	UFUNCTION(BlueprintPure, Category="UAV|Telemetry")
	float GetAirDensityAtPosition(FVector Pos) const;

private:
	FRigidBody6DOF      RigidBody;
	FAerodynamicsSolver AeroSolver;
	FMicroburstWindField WindField;

	FAeroForceResult    LastAeroResult;

	float AileronInput  = 0.0f;
	float ElevatorInput = 0.0f;
	float RudderInput   = 0.0f;
	float ThrottleInput = 0.0f;

	float PhysicsAccumulator = 0.0f;

	float MaxAileronMoment  = 50.0f;
	float MaxElevatorMoment = 80.0f;
	float MaxRudderMoment   = 30.0f;
	float MaxThrustForce    = 120.0f;

	bool bInitialized = false;
	bool bEmergencyStopped = false;

	void InitializePhysics();

	void FixedPhysicsTick_AdaptiveRK4(float FrameDt);
	void FixedPhysicsTick_RK4(float FixedDt);
	void FixedPhysicsTick_LegacyEuler(float FixedDt);

	void ApplyControlSurfaceForces();
	void ComputeAndApplyAerodynamics(float StepTime);

	void UpdateTelemetry();
	void UpdateSystemHealth();

	void DrawDebugVisualization();
	void DrawWindFieldVisualization();

	void InputThrottle(const FInputActionValue& Value);
	void InputPitch(const FInputActionValue& Value);
	void InputRoll(const FInputActionValue& Value);
	void InputYaw(const FInputActionValue& Value);
};
