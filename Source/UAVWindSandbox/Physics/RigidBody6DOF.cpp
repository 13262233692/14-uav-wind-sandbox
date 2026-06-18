#include "Physics/RigidBody6DOF.h"

FRigidBody6DOF::FRigidBody6DOF()
{
	Reset();
}

void FRigidBody6DOF::SetMass(float InMass)
{
	Mass = FMath::Max(InMass, 0.001f);
}

void FRigidBody6DOF::SetInertiaTensor(const FVector& InDiagonal)
{
	InertiaDiagonal.X = FMath::Max(InDiagonal.X, 0.0001f);
	InertiaDiagonal.Y = FMath::Max(InDiagonal.Y, 0.0001f);
	InertiaDiagonal.Z = FMath::Max(InDiagonal.Z, 0.0001f);
	InertiaInverse.X = 1.0f / InertiaDiagonal.X;
	InertiaInverse.Y = 1.0f / InertiaDiagonal.Y;
	InertiaInverse.Z = 1.0f / InertiaDiagonal.Z;
}

void FRigidBody6DOF::SetInitialState(const FVector& InPos, const FQuat& InQuat)
{
	State.Position = InPos;
	State.Orientation = InQuat.GetNormalized();
	State.LinVelocity = FVector::ZeroVector;
	State.AngVelocity = FVector::ZeroVector;
	State.LinMomentum = FVector::ZeroVector;
	State.AngMomentum = FVector::ZeroVector;
	AccumulatedForce = FVector::ZeroVector;
	AccumulatedTorque = FVector::ZeroVector;
}

void FRigidBody6DOF::Reset()
{
	SetInitialState(FVector::ZeroVector, FQuat::Identity);
}

void FRigidBody6DOF::AddForce_World(const FVector& Force)
{
	AccumulatedForce += Force;
}

void FRigidBody6DOF::AddTorque_World(const FVector& Torque)
{
	AccumulatedTorque += Torque;
}

void FRigidBody6DOF::AddForceAtPosition_World(const FVector& Force, const FVector& WorldPos)
{
	AccumulatedForce += Force;
	FVector Radius = WorldPos - State.Position;
	AccumulatedTorque += FVector::CrossProduct(Radius, Force);
}

FMatrix FRigidBody6DOF::GetBodyToWorldMatrix() const
{
	return FQuatRotationTranslationMatrix(State.Orientation, State.Position);
}

FMatrix FRigidBody6DOF::GetWorldToBodyMatrix() const
{
	return GetBodyToWorldMatrix().Inverse();
}

FVector FRigidBody6DOF::TransformWorldToBody(const FVector& WorldVec) const
{
	return State.Orientation.UnrotateVector(WorldVec);
}

FVector FRigidBody6DOF::TransformBodyToWorld(const FVector& BodyVec) const
{
	return State.Orientation.RotateVector(BodyVec);
}

void FRigidBody6DOF::UpdateVelocityFromMomentum()
{
	State.LinVelocity = State.LinMomentum / Mass;

	FVector BodyAngMom = TransformWorldToBody(State.AngMomentum);
	FVector BodyAngVel(
		BodyAngMom.X * InertiaInverse.X,
		BodyAngMom.Y * InertiaInverse.Y,
		BodyAngMom.Z * InertiaInverse.Z
	);
	State.AngVelocity = TransformBodyToWorld(BodyAngVel);
}

void FRigidBody6DOF::Integrate_Quaternion(float DeltaTime)
{
	FVector Omega = State.AngVelocity;
	float HalfDt = 0.5f * DeltaTime;

	FQuat OmegaQuat(
		Omega.X * HalfDt,
		Omega.Y * HalfDt,
		Omega.Z * HalfDt,
		0.0f
	);

	FQuat Derivative = OmegaQuat * State.Orientation;
	State.Orientation.X += Derivative.X;
	State.Orientation.Y += Derivative.Y;
	State.Orientation.Z += Derivative.Z;
	State.Orientation.W += Derivative.W;
	State.Orientation.Normalize();
}

void FRigidBody6DOF::Integrate(float DeltaTime, const FVector& Gravity)
{
	State.LinMomentum += (AccumulatedForce + Gravity * Mass) * DeltaTime;
	State.AngMomentum += AccumulatedTorque * DeltaTime;

	UpdateVelocityFromMomentum();

	State.Position += State.LinVelocity * DeltaTime;

	Integrate_Quaternion(DeltaTime);

	AccumulatedForce = FVector::ZeroVector;
	AccumulatedTorque = FVector::ZeroVector;
}
