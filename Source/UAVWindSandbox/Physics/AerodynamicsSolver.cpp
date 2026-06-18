#include "Physics/AerodynamicsSolver.h"
#include <cmath>

FAerodynamicsSolver::FAerodynamicsSolver()
{
	FAircraftConfig Default;
	Default.WingSpanM = 25.0f;
	Default.WingAreaM2 = 12.0f;
	Default.AspectRatio = (Default.WingSpanM * Default.WingSpanM) / Default.WingAreaM2;
	Initialize(Default);
}

void FAerodynamicsSolver::Initialize(const FAircraftConfig& Config)
{
	Aircraft = Config;
	FlexDeflection = 0.0f;
	FlexVelocity = 0.0f;
	TorsionDeflection = 0.0f;
	TorsionVelocity = 0.0f;
	FlutterTimeAccum = 0.0f;
	FlutterAmplitude = 0.0f;
	Health = EAeroSolverHealth::Healthy;
	EmergencyClampCount = 0;
}

void FAerodynamicsSolver::ResetHealth()
{
	Health = EAeroSolverHealth::Healthy;
}

void FAerodynamicsSolver::MarkHealth(EAeroSolverHealth NewHealth)
{
	if (NewHealth > Health)
	{
		Health = NewHealth;
	}
}

bool FAerodynamicsSolver::ValidateInputVector(const FVector& V) const
{
	return FPlatformMath::IsFinite(V.X)
		&& FPlatformMath::IsFinite(V.Y)
		&& FPlatformMath::IsFinite(V.Z);
}

bool FAerodynamicsSolver::ValidateFloat(float Val) const
{
	return FPlatformMath::IsFinite(Val);
}

float FAerodynamicsSolver::SafeDivide(float Numerator, float Denominator, float Default) const
{
	float AbsDenom = FMath::Abs(Denominator);
	if (AbsDenom < UAV_AERO_EPSILON)
	{
		MarkHealth(EAeroSolverHealth::InputClamped);
		return Default;
	}
	float Result = Numerator / Denominator;
	if (!ValidateFloat(Result))
	{
		MarkHealth(EAeroSolverHealth::OutputClamped);
		return Default;
	}
	return Result;
}

float FAerodynamicsSolver::SafeAsin(float X) const
{
	float ClampedX = FMath::Clamp(X, -0.999999f, 0.999999f);
	if (FMath::Abs(X) > 1.0f)
	{
		MarkHealth(EAeroSolverHealth::InputClamped);
	}
	return FMath::Asin(ClampedX);
}

float FAerodynamicsSolver::SafeAcos(float X) const
{
	float ClampedX = FMath::Clamp(X, -0.999999f, 0.999999f);
	if (FMath::Abs(X) > 1.0f)
	{
		MarkHealth(EAeroSolverHealth::InputClamped);
	}
	return FMath::Acos(ClampedX);
}

void FAerodynamicsSolver::ClampInput(FVector& Velocity, float& AirDensity, float& DeltaTime)
{
	bool bClamped = false;

	if (!ValidateInputVector(Velocity))
	{
		Velocity = FVector::ZeroVector;
		bClamped = true;
	}

	float VelMag = Velocity.Size();
	if (VelMag > UAV_AERO_MAX_AIRSPEED)
	{
		Velocity = Velocity.GetSafeNormal() * UAV_AERO_MAX_AIRSPEED;
		bClamped = true;
	}

	if (!ValidateFloat(AirDensity) || AirDensity < 0.05f)
	{
		AirDensity = 1.225f;
		bClamped = true;
	}
	AirDensity = FMath::Clamp(AirDensity, 0.05f, 1.5f);

	if (!ValidateFloat(DeltaTime) || DeltaTime <= 0.0f)
	{
		DeltaTime = 0.002f;
		bClamped = true;
	}
	DeltaTime = FMath::Clamp(DeltaTime, 0.00001f, 0.1f);

	if (bClamped)
	{
		MarkHealth(EAeroSolverHealth::InputClamped);
	}
}

void FAerodynamicsSolver::ComputeAirspeedAndAngles(
	const FVector& WorldVelocity,
	const FQuat&   BodyOrientation
)
{
	Airspeed_Body = BodyOrientation.UnrotateVector(WorldVelocity);

	if (!ValidateInputVector(Airspeed_Body))
	{
		Airspeed_Body = FVector::ZeroVector;
		MarkHealth(EAeroSolverHealth::InputClamped);
	}

	AirspeedMag = Airspeed_Body.Size();

	if (!ValidateFloat(AirspeedMag))
	{
		AirspeedMag = 0.0f;
		MarkHealth(EAeroSolverHealth::InputClamped);
	}

	if (AirspeedMag < UAV_AERO_MIN_AIRSPEED)
	{
		AlphaDeg = 0.0f;
		BetaDeg = 0.0f;
		return;
	}

	float U = Airspeed_Body.X;
	float V = Airspeed_Body.Y;
	float W = Airspeed_Body.Z;

	float UV_Plane = FMath::Sqrt(U * U + W * W);
	if (UV_Plane < UAV_AERO_EPSILON)
	{
		AlphaDeg = 0.0f;
	}
	else
	{
		float AlphaRad = FMath::Atan2(W, U);
		AlphaDeg = FMath::RadiansToDegrees(AlphaRad);
	}

	float BetaArg = SafeDivide(V, AirspeedMag, 0.0f);
	BetaDeg = FMath::RadiansToDegrees(SafeAsin(BetaArg));

	AlphaDeg = FMath::Clamp(AlphaDeg, -UAV_AERO_MAX_ALPHA_DEG, UAV_AERO_MAX_ALPHA_DEG);
	BetaDeg = FMath::Clamp(BetaDeg, -UAV_AERO_MAX_BETA_DEG, UAV_AERO_MAX_BETA_DEG);

	if (!ValidateFloat(AlphaDeg) || !ValidateFloat(BetaDeg))
	{
		AlphaDeg = 0.0f;
		BetaDeg = 0.0f;
		MarkHealth(EAeroSolverHealth::OutputClamped);
	}
}

void FAerodynamicsSolver::ComputeLiftDragCoefficients()
{
	float AlphaRad = FMath::DegreesToRadians(AlphaDeg);
	float StallAlphaRad = FMath::DegreesToRadians(Aircraft.StallAlphaDeg);

	float AR = Aircraft.AspectRatio;
	float e  = Aircraft.OswaldEfficiency;
	float Cla = Aircraft.ClAlphaPerRad;

	AR = FMath::Max(AR, 1.0f);
	e  = FMath::Clamp(e, 0.5f, 1.0f);
	Cla = FMath::Max(Cla, 1.0f);

	if (FMath::Abs(AlphaRad) < StallAlphaRad)
	{
		float Factor = 1.0f + 2.0f / AR;
		CL = Cla * AlphaRad / Factor;
	}
	else
	{
		float Sign = FMath::Sign(AlphaRad);
		float Excess = FMath::Abs(AlphaRad) - StallAlphaRad;
		float StallDrop = FMath::Exp(-Excess * 4.0f);
		float CLmax = Cla * StallAlphaRad / (1.0f + 2.0f / AR);
		CL = Sign * CLmax * StallDrop;
	}

	float CL_sq = CL * CL;
	float CDinduced = SafeDivide(CL_sq, PI * AR * e, 0.0f);
	float CdProfile = Aircraft.CdMin + 2.0f * (1.0f - FMath::Cos(2.0f * AlphaRad)) * 0.02f;
	CD = CDinduced + CdProfile;

	CM = -0.5f * CL;

	ClampCoefficients();
}

void FAerodynamicsSolver::ClampCoefficients()
{
	if (!ValidateFloat(CL))
	{
		CL = 0.0f;
		MarkHealth(EAeroSolverHealth::OutputClamped);
	}
	if (!ValidateFloat(CD))
	{
		CD = Aircraft.CdMin;
		MarkHealth(EAeroSolverHealth::OutputClamped);
	}
	if (!ValidateFloat(CM))
	{
		CM = 0.0f;
		MarkHealth(EAeroSolverHealth::OutputClamped);
	}

	CL = FMath::Clamp(CL, UAV_AERO_MIN_CL, UAV_AERO_MAX_CL);
	CD = FMath::Clamp(CD, 0.001f, UAV_AERO_MAX_CD);
	CM = FMath::Clamp(CM, -UAV_AERO_MAX_CM, UAV_AERO_MAX_CM);
}

void FAerodynamicsSolver::ComputeStabilityDerivatives(float b, float c, float S)
{
	float AlphaRad = FMath::DegreesToRadians(AlphaDeg);
	float BetaRad  = FMath::DegreesToRadians(BetaDeg);

	Cl = -0.15f * BetaRad;
	Cn =  0.08f * BetaRad;
	CY = -0.25f * BetaRad;

	float Denom = 0.5f * FMath::Max(AirspeedMag, UAV_AERO_MIN_AIRSPEED);
	float Denom_b = Denom * FMath::Max(b, 0.1f);
	float Denom_c = Denom * FMath::Max(c, 0.05f);

	float pBar = SafeDivide(0.0f, Denom_b, 0.0f);
	float qBar = SafeDivide(0.0f, Denom_c, 0.0f);
	float rBar = SafeDivide(0.0f, Denom_b, 0.0f);

	Cl += -0.3f * pBar;
	Cn += -0.05f * pBar;
	CM += -2.5f * qBar;
	Cl +=  0.05f * rBar;
	Cn += -0.12f * rBar;

	if (!ValidateFloat(Cl) || !ValidateFloat(Cn) || !ValidateFloat(CY))
	{
		Cl = 0.0f;
		Cn = 0.0f;
		CY = 0.0f;
		MarkHealth(EAeroSolverHealth::OutputClamped);
	}

	Cl = FMath::Clamp(Cl, -1.0f, 1.0f);
	Cn = FMath::Clamp(Cn, -1.0f, 1.0f);
	CY = FMath::Clamp(CY, -1.0f, 1.0f);
}

void FAerodynamicsSolver::ComputeFlutterForces(float DeltaTime, float AirDensity)
{
	FlutterTimeAccum += DeltaTime;
	FlutterPhase += 2.0f * PI * Aircraft.FlutterFrequencyHz * DeltaTime;

	float b = FMath::Max(Aircraft.WingSpanM * 0.5f, 0.1f);
	float S = FMath::Max(Aircraft.WingAreaM2, 0.1f);
	float MassKgf = FMath::Max(Aircraft.MassKg, 1.0f);

	float VelocityEffect = FMath::Min(AirspeedMag * AirspeedMag * 0.01f, 5.0f);
	if (!ValidateFloat(VelocityEffect)) VelocityEffect = 0.0f;

	float DynPress = 0.5f * AirDensity * FMath::Min(AirspeedMag * AirspeedMag, UAV_AERO_MAX_AIRSPEED * UAV_AERO_MAX_AIRSPEED);
	float EqAeroForce = DynPress * S * 0.02f * FMath::Sin(FlutterPhase) * VelocityEffect;

	float FlexSpringForce = -Aircraft.FlexuralRigidity * FlexDeflection;
	float DampingCoeff = 2.0f * Aircraft.FlutterDampingRatio
		* FMath::Sqrt(Aircraft.FlexuralRigidity * MassKgf);
	float FlexDampingForce = -DampingCoeff * FlexVelocity;

	float FlexAccel = SafeDivide(EqAeroForce + FlexSpringForce + FlexDampingForce, MassKgf, 0.0f);
	FlexVelocity += FlexAccel * DeltaTime;
	FlexDeflection += FlexVelocity * DeltaTime;

	float TorsionAeroMoment = DynPress * S * b * 0.015f * FMath::Sin(FlutterPhase + 0.8f) * VelocityEffect;

	float TorsionSpring = -Aircraft.TorsionalRigidity * TorsionDeflection;
	float TorsionDampCoeff = 2.0f * Aircraft.FlutterDampingRatio
		* FMath::Sqrt(Aircraft.TorsionalRigidity * FMath::Max(Aircraft.InertiaDiagonal.X, 0.1f));
	float TorsionDamp = -TorsionDampCoeff * TorsionVelocity;

	float TorsionInertia = FMath::Max(Aircraft.InertiaDiagonal.X * 0.3f, 0.01f);
	float TorsionAccel = SafeDivide(TorsionAeroMoment + TorsionSpring + TorsionDamp, TorsionInertia, 0.0f);
	TorsionVelocity += TorsionAccel * DeltaTime;
	TorsionDeflection += TorsionVelocity * DeltaTime;

	if (!ValidateFloat(FlexDeflection)) FlexDeflection = 0.0f;
	if (!ValidateFloat(FlexVelocity)) FlexVelocity = 0.0f;
	if (!ValidateFloat(TorsionDeflection)) TorsionDeflection = 0.0f;
	if (!ValidateFloat(TorsionVelocity)) TorsionVelocity = 0.0f;

	FlexDeflection = FMath::Clamp(FlexDeflection, -UAV_AERO_MAX_FLUTTER_AMP, UAV_AERO_MAX_FLUTTER_AMP);
	FlexVelocity = FMath::Clamp(FlexVelocity, -20.0f, 20.0f);
	TorsionDeflection = FMath::Clamp(TorsionDeflection, -UAV_AERO_MAX_FLUTTER_AMP, UAV_AERO_MAX_FLUTTER_AMP);
	TorsionVelocity = FMath::Clamp(TorsionVelocity, -20.0f, 20.0f);

	FlutterAmplitude = FMath::Abs(FlexDeflection) + FMath::Abs(TorsionDeflection) * b;
	FlutterAmplitude = FMath::Clamp(FlutterAmplitude, 0.0f, UAV_AERO_MAX_FLUTTER_AMP);

	if (!ValidateFloat(FlutterAmplitude))
	{
		FlutterAmplitude = 0.0f;
		MarkHealth(EAeroSolverHealth::OutputClamped);
	}
}

FVector FAerodynamicsSolver::ComputeLiftVector_Body() const
{
	float AlphaRad = FMath::DegreesToRadians(AlphaDeg);
	FVector LiftDir(-FMath::Sin(AlphaRad), 0.0f, FMath::Cos(AlphaRad));
	float LiftMag = DynPressure * Aircraft.WingAreaM2 * CL;

	if (!ValidateFloat(LiftMag)) LiftMag = 0.0f;
	LiftMag = FMath::Clamp(LiftMag, -UAV_AERO_MAX_FORCE, UAV_AERO_MAX_FORCE);

	return LiftDir * LiftMag;
}

FVector FAerodynamicsSolver::ComputeDragVector_Body() const
{
	if (AirspeedMag < UAV_AERO_MIN_AIRSPEED) return FVector::ZeroVector;

	FVector DragDir = -Airspeed_Body.GetSafeNormal();
	float DragMag = DynPressure * Aircraft.WingAreaM2 * CD;

	if (!ValidateFloat(DragMag)) DragMag = 0.0f;
	DragMag = FMath::Clamp(DragMag, 0.0f, UAV_AERO_MAX_FORCE);

	return DragDir * DragMag;
}

FVector FAerodynamicsSolver::ComputeSideforceVector_Body() const
{
	float SideMag = DynPressure * Aircraft.WingAreaM2 * CY;

	if (!ValidateFloat(SideMag)) SideMag = 0.0f;
	SideMag = FMath::Clamp(SideMag, -UAV_AERO_MAX_FORCE, UAV_AERO_MAX_FORCE);

	return FVector(0.0f, SideMag, 0.0f);
}

FVector FAerodynamicsSolver::ComputeMomentVector_Body(float b, float c, float S, float Q) const
{
	float RollMom  = Q * S * b * Cl;
	float PitchMom = Q * S * c * CM;
	float YawMom   = Q * S * b * Cn;

	if (!ValidateFloat(RollMom)) RollMom = 0.0f;
	if (!ValidateFloat(PitchMom)) PitchMom = 0.0f;
	if (!ValidateFloat(YawMom)) YawMom = 0.0f;

	RollMom  = FMath::Clamp(RollMom,  -UAV_AERO_MAX_TORQUE, UAV_AERO_MAX_TORQUE);
	PitchMom = FMath::Clamp(PitchMom, -UAV_AERO_MAX_TORQUE, UAV_AERO_MAX_TORQUE);
	YawMom   = FMath::Clamp(YawMom,   -UAV_AERO_MAX_TORQUE, UAV_AERO_MAX_TORQUE);

	return FVector(RollMom, PitchMom, YawMom);
}

FVector FAerodynamicsSolver::BodyForcesToWorld(const FVector& BodyForce, const FQuat& Orientation) const
{
	FVector World = Orientation.RotateVector(BodyForce);
	if (!ValidateInputVector(World))
	{
		return FVector::ZeroVector;
	}
	return World;
}

FVector FAerodynamicsSolver::BodyTorquesToWorld(const FVector& BodyTorque, const FQuat& Orientation) const
{
	FVector World = Orientation.RotateVector(BodyTorque);
	if (!ValidateInputVector(World))
	{
		return FVector::ZeroVector;
	}
	return World;
}

void FAerodynamicsSolver::ClampForceResult(FAeroForceResult& Result) const
{
	auto ClampVec = [this](FVector& V, float Max)
	{
		if (!ValidateInputVector(V)) V = FVector::ZeroVector;
		V.X = FMath::Clamp(V.X, -Max, Max);
		V.Y = FMath::Clamp(V.Y, -Max, Max);
		V.Z = FMath::Clamp(V.Z, -Max, Max);
	};

	ClampVec(Result.Lift_World,       UAV_AERO_MAX_FORCE);
	ClampVec(Result.Drag_World,       UAV_AERO_MAX_FORCE);
	ClampVec(Result.InducedTorque,    UAV_AERO_MAX_TORQUE);
	ClampVec(Result.FlutterTorque,    UAV_AERO_MAX_TORQUE * 0.5f);
	ClampVec(Result.TotalForce_World, UAV_AERO_MAX_FORCE);
	ClampVec(Result.TotalTorque_World, UAV_AERO_MAX_TORQUE);
}

FAeroForceResult FAerodynamicsSolver::ComputeAerodynamicForces(
	const FVector& WorldVelocity,
	const FVector& WorldAngVelocity,
	const FQuat&   BodyOrientation,
	float          AirDensity,
	float          DeltaTime
)
{
	Health = EAeroSolverHealth::Healthy;

	FVector ClampedVelocity = WorldVelocity;
	ClampInput(ClampedVelocity, AirDensity, DeltaTime);

	ComputeAirspeedAndAngles(ClampedVelocity, BodyOrientation);

	DynPressure = 0.5f * AirDensity * FMath::Min(AirspeedMag * AirspeedMag, UAV_AERO_MAX_AIRSPEED * UAV_AERO_MAX_AIRSPEED);
	DynPressure = FMath::Clamp(DynPressure, 0.0f, UAV_AERO_MAX_DYNPRESSURE);
	if (!ValidateFloat(DynPressure))
	{
		DynPressure = 0.0f;
		MarkHealth(EAeroSolverHealth::OutputClamped);
	}

	ComputeLiftDragCoefficients();

	float b = FMath::Max(Aircraft.WingSpanM, 0.1f);
	float c = FMath::Max(Aircraft.MeanAerodynamicChord, 0.05f);
	float S = FMath::Max(Aircraft.WingAreaM2, 0.1f);

	ComputeStabilityDerivatives(b, c, S);
	ComputeFlutterForces(DeltaTime, AirDensity);

	FAeroForceResult Result;

	FVector Lift_Body = ComputeLiftVector_Body();
	FVector Drag_Body = ComputeDragVector_Body();
	FVector Side_Body = ComputeSideforceVector_Body();
	FVector Moment_Body = ComputeMomentVector_Body(b, c, S, DynPressure);

	Result.Lift_World = BodyForcesToWorld(Lift_Body, BodyOrientation);
	Result.Drag_World = BodyForcesToWorld(Drag_Body, BodyOrientation);
	Result.TotalForce_World = Result.Lift_World + Result.Drag_World
		+ BodyForcesToWorld(Side_Body, BodyOrientation);
	Result.TotalTorque_World = BodyTorquesToWorld(Moment_Body, BodyOrientation);
	Result.InducedTorque = Result.TotalTorque_World;

	float FlutterTorqueMag = DynPressure * S * b * 0.005f * FlutterAmplitude * 5.0f;
	FlutterTorqueMag = FMath::Clamp(FlutterTorqueMag, 0.0f, UAV_AERO_MAX_TORQUE * 0.3f);

	FVector FlutterTorque_Body(
		FlutterTorqueMag * FMath::Sin(FlutterPhase),
		FlutterTorqueMag * 0.3f * FMath::Sin(FlutterPhase * 1.5f),
		FlutterTorqueMag * 0.1f * FMath::Sin(FlutterPhase * 2.0f)
	);
	Result.FlutterTorque = BodyTorquesToWorld(FlutterTorque_Body, BodyOrientation);
	Result.TotalTorque_World += Result.FlutterTorque;

	ClampForceResult(Result);

	if (Health >= EAeroSolverHealth::OutputClamped)
	{
		EmergencyClampCount++;
	}

	return Result;
}
