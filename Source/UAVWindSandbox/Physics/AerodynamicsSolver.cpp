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
}

void FAerodynamicsSolver::ComputeAirspeedAndAngles(
	const FVector& WorldVelocity,
	const FQuat&   BodyOrientation
)
{
	Airspeed_Body = BodyOrientation.UnrotateVector(WorldVelocity);
	AirspeedMag = Airspeed_Body.Size();

	if (AirspeedMag < 0.001f)
	{
		AlphaDeg = 0.0f;
		BetaDeg = 0.0f;
		return;
	}

	float U = Airspeed_Body.X;
	float V = Airspeed_Body.Y;
	float W = Airspeed_Body.Z;

	float UV_Plane = FMath::Sqrt(U * U + W * W);
	AlphaDeg = FMath::RadiansToDegrees(FMath::Atan2(W, U));
	BetaDeg  = FMath::RadiansToDegrees(FMath::Asin(V / AirspeedMag));
}

void FAerodynamicsSolver::ComputeLiftDragCoefficients()
{
	float AlphaRad = FMath::DegreesToRadians(AlphaDeg);
	float StallAlphaRad = FMath::DegreesToRadians(Aircraft.StallAlphaDeg);

	float AR = Aircraft.AspectRatio;
	float e  = Aircraft.OswaldEfficiency;

	float Cla = Aircraft.ClAlphaPerRad;

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

	float CDinduced = (CL * CL) / (PI * AR * e);
	float CdProfile = Aircraft.CdMin + 2.0f * (1.0f - FMath::Cos(2.0f * AlphaRad)) * 0.02f;
	CD = CDinduced + CdProfile;

	CM = -0.5f * CL;
}

void FAerodynamicsSolver::ComputeStabilityDerivatives()
{
	float AlphaRad = FMath::DegreesToRadians(AlphaDeg);
	float BetaRad  = FMath::DegreesToRadians(BetaDeg);
	float b = Aircraft.WingSpanM;
	float c = Aircraft.MeanAerodynamicChord;
	float S = Aircraft.WingAreaM2;

	if (AirspeedMag < 0.001f)
	{
		Cl = 0.0f;
		Cn = 0.0f;
		CY = 0.0f;
		return;
	}

	Cl = -0.15f * BetaRad;
	Cn = 0.08f * BetaRad;
	CY = -0.25f * BetaRad;

	float pBar = (Aircraft.InertiaDiagonal.Y * 0.0f) / (0.5f * AirspeedMag * b);
	float qBar = (0.0f) / (0.5f * AirspeedMag * c);
	float rBar = (0.0f) / (0.5f * AirspeedMag * b);

	float AR = Aircraft.AspectRatio;
	float e = Aircraft.OswaldEfficiency;

	Cl += -0.3f * pBar;
	Cn += -0.05f * pBar;

	CM += -2.5f * qBar;

	Cl +=  0.05f * rBar;
	Cn += -0.12f * rBar;
}

void FAerodynamicsSolver::ComputeFlutterForces(float DeltaTime, float AirDensity)
{
	FlutterTimeAccum += DeltaTime;
	FlutterPhase += 2.0f * PI * Aircraft.FlutterFrequencyHz * DeltaTime;

	float b = Aircraft.WingSpanM * 0.5f;
	float S = Aircraft.WingAreaM2;

	float VelocityEffect = FMath::Min(AirspeedMag * AirspeedMag * 0.01f, 5.0f);

	float EqAeroForce = 0.5f * AirDensity * AirspeedMag * AirspeedMag * S
		* 0.02f * FMath::Sin(FlutterPhase) * VelocityEffect;

	float FlexSpringForce = -Aircraft.FlexuralRigidity * FlexDeflection;
	float FlexDampingForce = -2.0f * Aircraft.FlutterDampingRatio
		* FMath::Sqrt(Aircraft.FlexuralRigidity * Aircraft.MassKg)
		* FlexVelocity;

	float FlexAccel = (EqAeroForce + FlexSpringForce + FlexDampingForce) / Aircraft.MassKg;
	FlexVelocity += FlexAccel * DeltaTime;
	FlexDeflection += FlexVelocity * DeltaTime;

	float TorsionAeroMoment = 0.5f * AirDensity * AirspeedMag * AirspeedMag * S * b
		* 0.015f * FMath::Sin(FlutterPhase + 0.8f) * VelocityEffect;

	float TorsionSpring = -Aircraft.TorsionalRigidity * TorsionDeflection;
	float TorsionDamp   = -2.0f * Aircraft.FlutterDampingRatio
		* FMath::Sqrt(Aircraft.TorsionalRigidity * Aircraft.InertiaDiagonal.X)
		* TorsionVelocity;

	float TorsionInertia = FMath::Max(Aircraft.InertiaDiagonal.X * 0.3f, 0.01f);
	float TorsionAccel = (TorsionAeroMoment + TorsionSpring + TorsionDamp) / TorsionInertia;
	TorsionVelocity += TorsionAccel * DeltaTime;
	TorsionDeflection += TorsionVelocity * DeltaTime;

	FlutterAmplitude = FMath::Abs(FlexDeflection) + FMath::Abs(TorsionDeflection) * b;
}

FVector FAerodynamicsSolver::ComputeLiftVector_Body() const
{
	float AlphaRad = FMath::DegreesToRadians(AlphaDeg);
	FVector LiftDir(-FMath::Sin(AlphaRad), 0.0f, FMath::Cos(AlphaRad));
	return LiftDir * (DynPressure * Aircraft.WingAreaM2 * CL);
}

FVector FAerodynamicsSolver::ComputeDragVector_Body() const
{
	if (AirspeedMag < 0.001f) return FVector::ZeroVector;
	FVector DragDir = -Airspeed_Body.GetSafeNormal();
	return DragDir * (DynPressure * Aircraft.WingAreaM2 * CD);
}

FVector FAerodynamicsSolver::ComputeSideforceVector_Body() const
{
	return FVector(0.0f, DynPressure * Aircraft.WingAreaM2 * CY, 0.0f);
}

FVector FAerodynamicsSolver::ComputeMomentVector_Body() const
{
	float b = Aircraft.WingSpanM;
	float c = Aircraft.MeanAerodynamicChord;
	float S = Aircraft.WingAreaM2;
	float Q = DynPressure;

	return FVector(
		Q * S * b * Cl,
		Q * S * c * CM,
		Q * S * b * Cn
	);
}

FVector FAerodynamicsSolver::BodyForcesToWorld(const FVector& BodyForce, const FQuat& Orientation) const
{
	return Orientation.RotateVector(BodyForce);
}

FVector FAerodynamicsSolver::BodyTorquesToWorld(const FVector& BodyTorque, const FQuat& Orientation) const
{
	return Orientation.RotateVector(BodyTorque);
}

FAeroForceResult FAerodynamicsSolver::ComputeAerodynamicForces(
	const FVector& WorldVelocity,
	const FVector& WorldAngVelocity,
	const FQuat&   BodyOrientation,
	float          AirDensity,
	float          DeltaTime
)
{
	ComputeAirspeedAndAngles(WorldVelocity, BodyOrientation);

	DynPressure = 0.5f * AirDensity * AirspeedMag * AirspeedMag;

	ComputeLiftDragCoefficients();
	ComputeStabilityDerivatives();
	ComputeFlutterForces(DeltaTime, AirDensity);

	FAeroForceResult Result;

	FVector Lift_Body = ComputeLiftVector_Body();
	FVector Drag_Body = ComputeDragVector_Body();
	FVector Side_Body = ComputeSideforceVector_Body();
	FVector Moment_Body = ComputeMomentVector_Body();

	Result.Lift_World = BodyForcesToWorld(Lift_Body, BodyOrientation);
	Result.Drag_World = BodyForcesToWorld(Drag_Body, BodyOrientation);
	Result.TotalForce_World = Result.Lift_World + Result.Drag_World
		+ BodyForcesToWorld(Side_Body, BodyOrientation);
	Result.TotalTorque_World = BodyTorquesToWorld(Moment_Body, BodyOrientation);

	float b = Aircraft.WingSpanM;
	float FlutterTorqueMag = DynPressure * Aircraft.WingAreaM2 * b * 0.005f
		* FlutterAmplitude * 5.0f;

	FVector FlutterTorque_Body(
		FlutterTorqueMag * FMath::Sin(FlutterPhase),
		FlutterTorqueMag * 0.3f * FMath::Sin(FlutterPhase * 1.5f),
		FlutterTorqueMag * 0.1f * FMath::Sin(FlutterPhase * 2.0f)
	);
	Result.FlutterTorque = BodyTorquesToWorld(FlutterTorque_Body, BodyOrientation);
	Result.InducedTorque = Result.TotalTorque_World;

	Result.TotalTorque_World += Result.FlutterTorque;

	return Result;
}
