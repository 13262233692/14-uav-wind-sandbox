#pragma once

#include "CoreMinimal.h"
#include "Math/Quat.h"
#include "Math/Vector.h"
#include "Math/RotationMatrix.h"
#include "Math/Matrix.h"

struct FRigidBodyState6DOF
{
	FVector  Position     = FVector::ZeroVector;
	FQuat    Orientation  = FQuat::Identity;
	FVector  LinVelocity  = FVector::ZeroVector;
	FVector  AngVelocity  = FVector::ZeroVector;

	FVector  LinMomentum  = FVector::ZeroVector;
	FVector  AngMomentum  = FVector::ZeroVector;
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

	void Integrate(float DeltaTime, const FVector& Gravity);

	FRigidBodyState6DOF GetState() const { return State; }
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

private:
	float   Mass            = 1.0f;
	FVector InertiaDiagonal = FVector(1.0f, 1.0f, 1.0f);
	FVector InertiaInverse  = FVector(1.0f, 1.0f, 1.0f);

	FRigidBodyState6DOF State;

	FVector AccumulatedForce = FVector::ZeroVector;
	FVector AccumulatedTorque = FVector::ZeroVector;

	void UpdateVelocityFromMomentum();
	void Integrate_Quaternion(float DeltaTime);
};
