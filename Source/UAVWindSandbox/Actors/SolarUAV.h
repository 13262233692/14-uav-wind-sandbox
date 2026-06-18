#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "Physics/RigidBody6DOF.h"
#include "Physics/AerodynamicsSolver.h"
#include "Physics/MicroburstWindField.h"
#include "SolarUAV.generated.h"

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
	float FixedPhysicsStep = 0.002f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="UAV|Physics")
	float GravityMagnitude = 9.81f;

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
	float DebugDrawScale = 0.02f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="UAV|Debug")
	float DebugWindArrowScale = 3.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="UAV|Telemetry")
	FVector CurrentWorldVelocity;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="UAV|Telemetry")
	FVector CurrentAirspeedVec;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="UAV|Telemetry")
	float CurrentAirspeed;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="UAV|Telemetry")
	float CurrentAlphaDeg;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="UAV|Telemetry")
	float CurrentBetaDeg;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="UAV|Telemetry")
	float CurrentDynamicPressure;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="UAV|Telemetry")
	float CurrentCL;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="UAV|Telemetry")
	float CurrentCD;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="UAV|Telemetry")
	FVector CurrentWindVelocity;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="UAV|Telemetry")
	FVector CurrentWindShear;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="UAV|Telemetry")
	float CurrentMicroburstIntensity;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="UAV|Telemetry")
	float CurrentFlutterAmplitude;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="UAV|Telemetry")
	float SimElapsed;

	UFUNCTION(BlueprintCallable, Category="UAV|Control")
	void SetControlSurfaces(float Aileron, float Elevator, float Rudder, float Throttle);

	UFUNCTION(BlueprintCallable, Category="UAV|Control")
	void ResetToInitial(FVector Position, FRotator Orientation);

	UFUNCTION(BlueprintCallable, Category="UAV|WindField")
	void TriggerMicroburst(FVector Center, float Intensity);

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

	void InitializePhysics();
	void FixedPhysicsTick(float FixedDt);
	void ApplyControlSurfaceForces();
	void UpdateTelemetry();
	void DrawDebugVisualization();
	void DrawWindFieldVisualization();

	void InputThrottle(const FInputActionValue& Value);
	void InputPitch(const FInputActionValue& Value);
	void InputRoll(const FInputActionValue& Value);
	void InputYaw(const FInputActionValue& Value);
};
