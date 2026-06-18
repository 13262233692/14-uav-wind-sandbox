#pragma once

#include "CoreMinimal.h"
#include "Math/Vector.h"
#include "Math/Quat.h"
#include "Physics/RigidBody6DOF.h"

struct FWingSection
{
	float SpanFraction   = 0.0f;
	float Chord          = 1.0f;
	float TwistAngleDeg  = 0.0f;
	float IncidenceDeg   = 0.0f;
	float DihedralDeg    = 0.0f;
	float AirfoilClMax  = 1.4f;
	float AirfoilCd0    = 0.012f;
	float AirfoilCm0    = 0.0f;
	float AlphaStallDeg = 16.0f;
};

struct FAircraftConfig
{
	float  MassKg              = 50.0f;
	float  WingSpanM           = 25.0f;
	float  WingAreaM2          = 12.0f;
	float  AspectRatio         = 52.0f;
	float  OswaldEfficiency    = 0.92f;
	float  MeanAerodynamicChord = 0.48f;

	FVector InertiaDiagonal    = FVector(85.0f, 8.0f, 90.0f);
	FVector CGBodyOffset       = FVector(0.0f, 0.0f, -0.05f);

	TArray<FWingSection> WingSections;

	float  FlutterFrequencyHz   = 3.5f;
	float  FlutterDampingRatio  = 0.15f;
	float  FlexuralRigidity     = 2500.0f;
	float  TorsionalRigidity    = 180.0f;

	float  StallAlphaDeg        = 16.0f;
	float  ClAlphaPerRad        = 5.8f;
	float  CdMin                = 0.015f;
};

class UAVWINDSANDBOX_API FAerodynamicsSolver
{
public:
	FAerodynamicsSolver();

	void Initialize(const FAircraftConfig& Config);

	FAeroForceResult ComputeAerodynamicForces(
		const FVector& WorldVelocity,
		const FVector& WorldAngVelocity,
		const FQuat&   BodyOrientation,
		float          AirDensity,
		float          DeltaTime
	);

	FAircraftConfig GetAircraftConfig() const { return Aircraft; }
	void SetAircraftConfig(const FAircraftConfig& InConfig) { Aircraft = InConfig; }

	FVector GetAirspeedVector_Body() const { return Airspeed_Body; }
	float   GetAirspeedMagnitude() const { return AirspeedMag; }
	float   GetAlphaDeg() const { return AlphaDeg; }
	float   GetBetaDeg() const { return BetaDeg; }
	float   GetDynamicPressure() const { return DynPressure; }
	float   GetLiftCoefficient() const { return CL; }
	float   GetDragCoefficient() const { return CD; }
	float   GetFlutterAmplitude() const { return FlutterAmplitude; }

private:
	FAircraftConfig Aircraft;

	FVector Airspeed_Body = FVector::ZeroVector;
	float   AirspeedMag   = 0.0f;
	float   AlphaDeg      = 0.0f;
	float   BetaDeg       = 0.0f;
	float   DynPressure   = 0.0f;
	float   CL            = 0.0f;
	float   CD            = 0.0f;
	float   CM            = 0.0f;
	float   CY            = 0.0f;
	float   Cl            = 0.0f;
	float   Cn            = 0.0f;

	float   FlutterAmplitude   = 0.0f;
	float   FlutterPhase       = 0.0f;
	float   FlexDeflection     = 0.0f;
	float   FlexVelocity       = 0.0f;
	float   TorsionDeflection  = 0.0f;
	float   TorsionVelocity    = 0.0f;
	float   FlutterTimeAccum   = 0.0f;

	void ComputeAirspeedAndAngles(
		const FVector& WorldVelocity,
		const FQuat&   BodyOrientation
	);

	void ComputeLiftDragCoefficients();
	void ComputeStabilityDerivatives();
	void ComputeFlutterForces(float DeltaTime, float AirDensity);

	FVector ComputeLiftVector_Body() const;
	FVector ComputeDragVector_Body() const;
	FVector ComputeSideforceVector_Body() const;
	FVector ComputeMomentVector_Body() const;

	FVector BodyForcesToWorld(const FVector& BodyForce, const FQuat& Orientation) const;
	FVector BodyTorquesToWorld(const FVector& BodyTorque, const FQuat& Orientation) const;
};
