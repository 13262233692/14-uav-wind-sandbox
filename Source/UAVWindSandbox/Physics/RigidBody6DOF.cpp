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

	LastValidState = State;
	Health = EIntegratorHealth::Healthy;
	EmergencyRecoveryCount = 0;
	Stats = FIntegrationStats();
}

void FRigidBody6DOF::Reset()
{
	SetInitialState(FVector::ZeroVector, FQuat::Identity);
}

void FRigidBody6DOF::AddForce_World(const FVector& Force)
{
	if (IsVectorValid(Force))
	{
		AccumulatedForce += Force;
	}
}

void FRigidBody6DOF::AddTorque_World(const FVector& Torque)
{
	if (IsVectorValid(Torque))
	{
		AccumulatedTorque += Torque;
	}
}

void FRigidBody6DOF::AddForceAtPosition_World(const FVector& Force, const FVector& WorldPos)
{
	if (IsVectorValid(Force) && IsVectorValid(WorldPos))
	{
		AccumulatedForce += Force;
		FVector Radius = WorldPos - State.Position;
		FVector Torque = FVector::CrossProduct(Radius, Force);
		if (IsVectorValid(Torque))
		{
			AccumulatedTorque += Torque;
		}
	}
}

void FRigidBody6DOF::ClearForcesAndTorques()
{
	AccumulatedForce = FVector::ZeroVector;
	AccumulatedTorque = FVector::ZeroVector;
}

void FRigidBody6DOF::SetState(const FRigidBodyState6DOF& NewState)
{
	State = NewState;
	if (ValidateState(State))
	{
		ClampState(State);
		UpdateVelocityFromMomentum(State);
		LastValidState = State;
	}
	else
	{
		RecoverFromInvalidState();
	}
}

bool FRigidBody6DOF::IsVectorValid(const FVector& V) const
{
	return FPlatformMath::IsFinite(V.X)
		&& FPlatformMath::IsFinite(V.Y)
		&& FPlatformMath::IsFinite(V.Z);
}

bool FRigidBody6DOF::IsQuatValid(const FQuat& Q) const
{
	return FPlatformMath::IsFinite(Q.X)
		&& FPlatformMath::IsFinite(Q.Y)
		&& FPlatformMath::IsFinite(Q.Z)
		&& FPlatformMath::IsFinite(Q.W)
		&& Q.SizeSquared() > 1e-10f;
}

bool FRigidBody6DOF::ValidateState(const FRigidBodyState6DOF& InState) const
{
	return IsVectorValid(InState.Position)
		&& IsVectorValid(InState.LinVelocity)
		&& IsVectorValid(InState.AngVelocity)
		&& IsVectorValid(InState.LinMomentum)
		&& IsVectorValid(InState.AngMomentum)
		&& IsQuatValid(InState.Orientation);
}

void FRigidBody6DOF::ClampState(FRigidBodyState6DOF& InOutState) const
{
	InOutState.Position.X = FMath::Clamp(InOutState.Position.X, -UAV_STATE_CLAMP_POS, UAV_STATE_CLAMP_POS);
	InOutState.Position.Y = FMath::Clamp(InOutState.Position.Y, -UAV_STATE_CLAMP_POS, UAV_STATE_CLAMP_POS);
	InOutState.Position.Z = FMath::Clamp(InOutState.Position.Z, -UAV_STATE_CLAMP_POS, UAV_STATE_CLAMP_POS);

	InOutState.LinVelocity.X = FMath::Clamp(InOutState.LinVelocity.X, -UAV_STATE_CLAMP_LINVEL, UAV_STATE_CLAMP_LINVEL);
	InOutState.LinVelocity.Y = FMath::Clamp(InOutState.LinVelocity.Y, -UAV_STATE_CLAMP_LINVEL, UAV_STATE_CLAMP_LINVEL);
	InOutState.LinVelocity.Z = FMath::Clamp(InOutState.LinVelocity.Z, -UAV_STATE_CLAMP_LINVEL, UAV_STATE_CLAMP_LINVEL);

	InOutState.AngVelocity.X = FMath::Clamp(InOutState.AngVelocity.X, -UAV_STATE_CLAMP_ANGVEL, UAV_STATE_CLAMP_ANGVEL);
	InOutState.AngVelocity.Y = FMath::Clamp(InOutState.AngVelocity.Y, -UAV_STATE_CLAMP_ANGVEL, UAV_STATE_CLAMP_ANGVEL);
	InOutState.AngVelocity.Z = FMath::Clamp(InOutState.AngVelocity.Z, -UAV_STATE_CLAMP_ANGVEL, UAV_STATE_CLAMP_ANGVEL);

	InOutState.Orientation.Normalize();
}

void FRigidBody6DOF::RecoverFromInvalidState()
{
	State = LastValidState;
	AccumulatedForce = FVector::ZeroVector;
	AccumulatedTorque = FVector::ZeroVector;
	Health = EIntegratorHealth::StateRecovered;
	EmergencyRecoveryCount++;
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

void FRigidBody6DOF::UpdateVelocityFromMomentum(FRigidBodyState6DOF& InOutState) const
{
	InOutState.LinVelocity = InOutState.LinMomentum / Mass;

	FVector BodyAngMom = InOutState.Orientation.UnrotateVector(InOutState.AngMomentum);
	FVector BodyAngVel(
		BodyAngMom.X * InertiaInverse.X,
		BodyAngMom.Y * InertiaInverse.Y,
		BodyAngMom.Z * InertiaInverse.Z
	);
	InOutState.AngVelocity = InOutState.Orientation.RotateVector(BodyAngVel);
}

FRigidBodyDerivative6DOF FRigidBody6DOF::ComputeQuaternionDerivative(
	const FQuat& Orientation,
	const FVector& AngularVelocity
) const
{
	FRigidBodyDerivative6DOF Deriv;
	float Half = 0.5f;

	FQuat OmegaQuat(
		AngularVelocity.X * Half,
		AngularVelocity.Y * Half,
		AngularVelocity.Z * Half,
		0.0f
	);

	Deriv.dOrientation_dt = OmegaQuat * Orientation;
	return Deriv;
}

FRigidBodyDerivative6DOF FRigidBody6DOF::ComputeDerivatives(
	const FRigidBodyState6DOF& CurrentState,
	const FVector& Gravity
) const
{
	FRigidBodyDerivative6DOF Deriv;

	FRigidBodyState6DOF StateCopy = CurrentState;
	UpdateVelocityFromMomentum(StateCopy);

	Deriv.dPosition_dt = StateCopy.LinVelocity;

	FRigidBodyDerivative6DOF QuatDeriv = ComputeQuaternionDerivative(
		StateCopy.Orientation,
		StateCopy.AngVelocity
	);
	Deriv.dOrientation_dt = QuatDeriv.dOrientation_dt;

	FVector TotalForce = AccumulatedForce + Gravity * Mass;
	Deriv.dLinMomentum_dt = TotalForce;

	Deriv.dAngMomentum_dt = AccumulatedTorque;

	return Deriv;
}

FRigidBodyDerivative6DOF FRigidBody6DOF::AddDerivatives(
	const FRigidBodyDerivative6DOF& A,
	const FRigidBodyDerivative6DOF& B,
	float Scale
) const
{
	FRigidBodyDerivative6DOF Result;
	Result.dPosition_dt    = A.dPosition_dt + B.dPosition_dt * Scale;
	Result.dLinMomentum_dt = A.dLinMomentum_dt + B.dLinMomentum_dt * Scale;
	Result.dAngMomentum_dt = A.dAngMomentum_dt + B.dAngMomentum_dt * Scale;
	Result.dOrientation_dt.X = A.dOrientation_dt.X + B.dOrientation_dt.X * Scale;
	Result.dOrientation_dt.Y = A.dOrientation_dt.Y + B.dOrientation_dt.Y * Scale;
	Result.dOrientation_dt.Z = A.dOrientation_dt.Z + B.dOrientation_dt.Z * Scale;
	Result.dOrientation_dt.W = A.dOrientation_dt.W + B.dOrientation_dt.W * Scale;
	return Result;
}

FRigidBodyState6DOF FRigidBody6DOF::AdvanceState(
	const FRigidBodyState6DOF& S,
	const FRigidBodyDerivative6DOF& D,
	float StepTime
) const
{
	FRigidBodyState6DOF NewState;
	NewState.Position    = S.Position + D.dPosition_dt * StepTime;
	NewState.LinMomentum = S.LinMomentum + D.dLinMomentum_dt * StepTime;
	NewState.AngMomentum = S.AngMomentum + D.dAngMomentum_dt * StepTime;

	NewState.Orientation.X = S.Orientation.X + D.dOrientation_dt.X * StepTime;
	NewState.Orientation.Y = S.Orientation.Y + D.dOrientation_dt.Y * StepTime;
	NewState.Orientation.Z = S.Orientation.Z + D.dOrientation_dt.Z * StepTime;
	NewState.Orientation.W = S.Orientation.W + D.dOrientation_dt.W * StepTime;
	NewState.Orientation.Normalize();

	UpdateVelocityFromMomentum(NewState);

	return NewState;
}

FRigidBodyState6DOF FRigidBody6DOF::RK4_Step(
	const FRigidBodyState6DOF& Initial,
	const FVector& Gravity,
	float StepTime
) const
{
	FRigidBodyDerivative6DOF k1 = ComputeDerivatives(Initial, Gravity);
	FRigidBodyState6DOF State_k2 = AdvanceState(Initial, k1, StepTime * 0.5f);

	FRigidBodyDerivative6DOF k2 = ComputeDerivatives(State_k2, Gravity);
	FRigidBodyState6DOF State_k3 = AdvanceState(Initial, k2, StepTime * 0.5f);

	FRigidBodyDerivative6DOF k3 = ComputeDerivatives(State_k3, Gravity);
	FRigidBodyState6DOF State_k4 = AdvanceState(Initial, k3, StepTime);

	FRigidBodyDerivative6DOF k4 = ComputeDerivatives(State_k4, Gravity);

	FRigidBodyDerivative6DOF Combined;
	Combined.dPosition_dt = (k1.dPosition_dt + 2.0f * (k2.dPosition_dt + k3.dPosition_dt) + k4.dPosition_dt) / 6.0f;
	Combined.dLinMomentum_dt = (k1.dLinMomentum_dt + 2.0f * (k2.dLinMomentum_dt + k3.dLinMomentum_dt) + k4.dLinMomentum_dt) / 6.0f;
	Combined.dAngMomentum_dt = (k1.dAngMomentum_dt + 2.0f * (k2.dAngMomentum_dt + k3.dAngMomentum_dt) + k4.dAngMomentum_dt) / 6.0f;
	Combined.dOrientation_dt.X = (k1.dOrientation_dt.X + 2.0f * (k2.dOrientation_dt.X + k3.dOrientation_dt.X) + k4.dOrientation_dt.X) / 6.0f;
	Combined.dOrientation_dt.Y = (k1.dOrientation_dt.Y + 2.0f * (k2.dOrientation_dt.Y + k3.dOrientation_dt.Y) + k4.dOrientation_dt.Y) / 6.0f;
	Combined.dOrientation_dt.Z = (k1.dOrientation_dt.Z + 2.0f * (k2.dOrientation_dt.Z + k3.dOrientation_dt.Z) + k4.dOrientation_dt.Z) / 6.0f;
	Combined.dOrientation_dt.W = (k1.dOrientation_dt.W + 2.0f * (k2.dOrientation_dt.W + k3.dOrientation_dt.W) + k4.dOrientation_dt.W) / 6.0f;

	FRigidBodyState6DOF Result = AdvanceState(Initial, Combined, StepTime);
	return Result;
}

float FRigidBody6DOF::ComputeStateError(
	const FRigidBodyState6DOF& FullStep,
	const FRigidBodyState6DOF& TwoHalfSteps
) const
{
	float PosError = FVector::Dist(FullStep.Position, TwoHalfSteps.Position) / UAV_STATE_CLAMP_POS;
	float LinMomError = FVector::Dist(FullStep.LinMomentum, TwoHalfSteps.LinMomentum) / (Mass * UAV_STATE_CLAMP_LINVEL + 1e-6f);
	float AngMomError = FVector::Dist(FullStep.AngMomentum, TwoHalfSteps.AngMomentum) / (InertiaDiagonal.Size() * UAV_STATE_CLAMP_ANGVEL + 1e-6f);

	float QX = FMath::Abs(FullStep.Orientation.X - TwoHalfSteps.Orientation.X);
	float QY = FMath::Abs(FullStep.Orientation.Y - TwoHalfSteps.Orientation.Y);
	float QZ = FMath::Abs(FullStep.Orientation.Z - TwoHalfSteps.Orientation.Z);
	float QW = FMath::Abs(FullStep.Orientation.W - TwoHalfSteps.Orientation.W);
	float QuatError = FMath::Max4(QX, QY, QZ, QW) / (PI + 1e-6f);

	return FMath::Max4(PosError, LinMomError, AngMomError, QuatError);
}

void FRigidBody6DOF::Integrate_RK4(float DeltaTime, const FVector& Gravity)
{
	State = RK4_Step(State, Gravity, DeltaTime);

	if (!ValidateState(State))
	{
		ClampState(State);
		if (!ValidateState(State))
		{
			RecoverFromInvalidState();
		}
	}

	if (Health == EIntegratorHealth::StateRecovered)
	{
		Health = EIntegratorHealth::Warning;
	}
	else if (Health == EIntegratorHealth::Warning)
	{
		Health = EIntegratorHealth::Healthy;
	}

	LastValidState = State;

	ClearForcesAndTorques();
}

void FRigidBody6DOF::Integrate_AdaptiveRK4(float& InOutDeltaTime, const FVector& Gravity)
{
	float RemainingTime = InOutDeltaTime;
	float CurrentStep = Stats.CurrentStepSize;

	Stats.SubStepsTaken = 0;
	Stats.RejectedSteps = 0;

	FRigidBodyState6DOF InitialState = State;

	while (RemainingTime > UAV_MIN_STEP_TIME * 0.5f)
	{
		CurrentStep = FMath::Clamp(CurrentStep, UAV_MIN_STEP_TIME, FMath::Min(UAV_MAX_STEP_TIME, RemainingTime));

		FRigidBodyState6DOF FullStepState = RK4_Step(State, Gravity, CurrentStep);
		FRigidBodyState6DOF HalfStep1 = RK4_Step(State, Gravity, CurrentStep * 0.5f);
		FRigidBodyState6DOF TwoHalfSteps = RK4_Step(HalfStep1, Gravity, CurrentStep * 0.5f);

		float Error = ComputeStateError(FullStepState, TwoHalfSteps);
		Stats.LastErrorEstimate = Error;

		if (!ValidateState(TwoHalfSteps))
		{
			ClampState(TwoHalfSteps);
			if (!ValidateState(TwoHalfSteps))
			{
				State = InitialState;
				RecoverFromInvalidState();
				InOutDeltaTime = 0.0f;
				Stats.bLastStepAccepted = false;
				return;
			}
		}

		float ErrorRatio = Error / (UAV_TARGET_ERROR + 1e-10f);
		float SafetyFactor = 0.9f;

		if (ErrorRatio <= 1.0f)
		{
			State = TwoHalfSteps;
			RemainingTime -= CurrentStep;
			Stats.SubStepsTaken++;
			Stats.bLastStepAccepted = true;

			float ScaleFactor = SafetyFactor * FMath::Pow(1.0f / FMath::Max(ErrorRatio, 1e-4f), 0.2f);
			CurrentStep *= FMath::Clamp(ScaleFactor, 0.5f, 2.0f);
			Health = EIntegratorHealth::Healthy;
		}
		else
		{
			Stats.RejectedSteps++;
			Stats.bLastStepAccepted = false;
			Health = EIntegratorHealth::Warning;

			float ScaleFactor = SafetyFactor * FMath::Pow(1.0f / FMath::Max(ErrorRatio, 1e-4f), 0.25f);
			CurrentStep *= FMath::Clamp(ScaleFactor, 0.2f, 0.8f);

			if (CurrentStep < UAV_MIN_STEP_TIME)
			{
				State = TwoHalfSteps;
				RemainingTime -= CurrentStep;
				Stats.SubStepsTaken++;
				CurrentStep = UAV_MIN_STEP_TIME;
			}
		}

		if (Stats.RejectedSteps > 100 || Stats.SubStepsTaken > 1000)
		{
			Health = EIntegratorHealth::EmergencyStop;
			State = InitialState;
			RecoverFromInvalidState();
			break;
		}
	}

	Stats.CurrentStepSize = FMath::Clamp(CurrentStep, UAV_MIN_STEP_TIME, UAV_MAX_STEP_TIME);
	LastValidState = State;
	ClearForcesAndTorques();

	InOutDeltaTime = InOutDeltaTime - RemainingTime;
}
