#pragma once

#include "CoreMinimal.h"
#include "Math/Quat.h"
#include "Math/Vector.h"
#include "Math/RotationMatrix.h"
#include "Math/Matrix.h"
#include "HAL/PlatformMath.h"

#define UAV_SAFE_DIV_EPSILON   1e-7f
#define UAV_STATE_CLAMP_LINVEL 300.0f
#define UAV_STATE_CLAMP_ANGVEL 50.0f
#define UAV_STATE_CLAMP_POS    100000.0f
#define UAV_MIN_STEP_TIME      0.00025f
#define UAV_MAX_STEP_TIME      0.005f
#define UAV_TARGET_ERROR       1e-5f

struct FRigidBodyState6DOF
{
	FVector  Position     = FVector::ZeroVector;
	FQuat    Orientation  = FQuat::Identity;
	FVector  LinMomentum  = FVector::ZeroVector;
	FVector  AngMomentum  = FVector::ZeroVector;

	FVector  LinVelocity  = FVector::ZeroVector;
	FVector  AngVelocity  = FVector::ZeroVector;
};

struct FRigidBodyDerivative6DOF
{
	FVector  dPosition_dt    = FVector::ZeroVector;
	FQuat    dOrientation_dt = FQuat(0,0,0,0);
	FVector  dLinMomentum_dt = FVector::ZeroVector;
	FVector  dAngMomentum_dt = FVector::ZeroVector;
};

struct FAeroForceResult
{
	FVector Lift_World       = FVector::ZeroVector;
	FVector Drag_World       = FVector::ZeroVector;
	FVector InducedTorque    = FVector::ZeroVector;
	FVector FlutterTorque    = FVector::ZeroVector;
	FVector TotalForce_World = FVector::ZeroVector;
	FVector TotalTorque_World = FVector::ZeroVector;
};

struct FIntegrationStats
{
	int32  SubStepsTaken   = 0;
	int32  RejectedSteps   = 0;
	float  CurrentStepSize = UAV_MIN_STEP_TIME;
	float  LastErrorEstimate = 0.0f;
	bool   bLastStepAccepted = true;
};

enum class EIntegratorHealth : uint8
{
	Healthy          = 0,
	Warning          = 1,
	StateRecovered   = 2,
	EmergencyStop    = 3
};

class UAVWINDSANDBOX_API FRigidBody6DOF
{
public:
	FRigidBody6DOF();

	void SetMass(float InMass);
	void SetInertiaTensor(const FVector& InDiagonal);
	void SetInitialState(const FVector& InPos, const FQuat& InQuat);

	void Reset();

	void AddForce_World(const FVector& Force);
	void AddTorque_World(const FVector& Torque);
	void AddForceAtPosition_World(const FVector& Force, const FVector& WorldPos);

	void Integrate_RK4(float DeltaTime, const FVector& Gravity);
	void Integrate_AdaptiveRK4(float& InOutDeltaTime, const FVector& Gravity);

	FRigidBodyState6DOF GetState() const { return State; }
	FRigidBodyState6DOF GetPreviousValidState() const { return LastValidState; }
	FVector GetWorldPosition() const { return State.Position; }
	FQuat   GetWorldOrientation() const { return State.Orientation; }
	FVector GetLinearVelocity() const { return State.LinVelocity; }
	FVector GetAngularVelocity() const { return State.AngVelocity; }

	FMatrix GetBodyToWorldMatrix() const;
	FMatrix GetWorldToBodyMatrix() const;

	FVector TransformWorldToBody(const FVector& WorldVec) const;
	FVector TransformBodyToWorld(const FVector& BodyVec) const;

	float GetMass() const { return Mass; }
	FVector GetInertiaDiagonal() const { return InertiaDiagonal; }

	FIntegrationStats GetIntegrationStats() const { return Stats; }
	EIntegratorHealth GetHealthStatus() const { return Health; }
	int32 GetTotalEmergencyRecoveries() const { return EmergencyRecoveryCount; }
	void ResetIntegrationStats() { Stats = FIntegrationStats(); }

	void ClearForcesAndTorques();

	void SetState(const FRigidBodyState6DOF& NewState);

	FRigidBodyDerivative6DOF ComputeDerivatives(
		const FRigidBodyState6DOF& CurrentState,
		const FVector& Gravity
	) const;

	FRigidBodyState6DOF AdvanceState(
		const FRigidBodyState6DOF& InState,
		const FRigidBodyDerivative6DOF& Deriv,
		float StepTime
	) const;

private:
	float   Mass            = 1.0f;
	FVector InertiaDiagonal = FVector(1.0f, 1.0f, 1.0f);
	FVector InertiaInverse  = FVector(1.0f, 1.0f, 1.0f);

	FRigidBodyState6DOF State;
	FRigidBodyState6DOF LastValidState;

	FVector AccumulatedForce = FVector::ZeroVector;
	FVector AccumulatedTorque = FVector::ZeroVector;

	FIntegrationStats Stats;
	EIntegratorHealth Health = EIntegratorHealth::Healthy;
	int32 EmergencyRecoveryCount = 0;

	void UpdateVelocityFromMomentum(FRigidBodyState6DOF& InOutState) const;

	FRigidBodyDerivative6DOF ComputeDerivatives(
		const FRigidBodyState6DOF& CurrentState,
		const FVector& Gravity
	) const;

	FRigidBodyState6DOF RK4_Step(
		const FRigidBodyState6DOF& Initial,
		const FVector& Gravity,
		float StepTime
	) const;

	FRigidBodyDerivative6DOF AddDerivatives(
		const FRigidBodyDerivative6DOF& A,
		const FRigidBodyDerivative6DOF& B,
		float Scale = 1.0f
	) const;

	FRigidBodyState6DOF AdvanceState(
		const FRigidBodyState6DOF& State,
		const FRigidBodyDerivative6DOF& Deriv,
		float StepTime
	) const;

	float ComputeStateError(
		const FRigidBodyState6DOF& FullStep,
		const FRigidBodyState6DOF& TwoHalfSteps
	) const;

	FRigidBodyDerivative6DOF ComputeQuaternionDerivative(
		const FQuat& Orientation,
		const FVector& AngularVelocity
	) const;

	bool ValidateState(const FRigidBodyState6DOF& InState) const;
	void ClampState(FRigidBodyState6DOF& InOutState) const;
	bool IsVectorValid(const FVector& V) const;
	bool IsQuatValid(const FQuat& Q) const;

	void RecoverFromInvalidState();
};
