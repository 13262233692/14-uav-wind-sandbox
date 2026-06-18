#pragma once

#include "CoreMinimal.h"
#include "Math/Vector.h"
#include "Math/Box.h"

struct FWindCell
{
	FVector Velocity = FVector::ZeroVector;
	float   Density  = 1.225f;
	float   Pressure = 101325.0f;
};

struct FMicroburstConfig
{
	FVector CenterPosition   = FVector(0.0f, 0.0f, 500.0f);
	float   MaxDowndraftSpeed = 25.0f;
	float   MaxOutflowSpeed   = 35.0f;
	float   CoreRadius        = 400.0f;
	float   OutflowRadius     = 1200.0f;
	float   HeightTop         = 1500.0f;
	float   HeightBottom      = 50.0f;
	float   EvolutionTime     = 120.0f;
	float   StartTime         = 30.0f;
	float   PeakTime          = 80.0f;
	float   ShearIntensity    = 1.0f;
	float   TurbulenceGain    = 0.3f;
};

class UAVWINDSANDBOX_API FMicroburstWindField
{
public:
	FMicroburstWindField();

	void Initialize(const FBox& WorldBounds, float CellSizeMeters);
	void SetMicroburstConfig(const FMicroburstConfig& Config);
	void SetAmbientWind(const FVector& WindVel) { AmbientWind = WindVel; }

	void UpdateField(float SimTime, float DeltaTime);

	FVector SampleWindVelocity(const FVector& WorldPosition) const;
	float   SampleAirDensity(const FVector& WorldPosition) const;
	FVector SampleWindShear(const FVector& WorldPosition) const;

	FVector GetAmbientWind() const { return AmbientWind; }
	FMicroburstConfig GetMicroburstConfig() const { return BurstCfg; }
	float   GetSimTime() const { return CurrentTime; }

	bool IsInsideMicroburst(const FVector& WorldPosition) const;
	float GetMicroburstIntensityAtTime(float SimTime) const;

private:
	TArray<FWindCell> Grid;
	FVector   GridMin;
	FVector   GridMax;
	FIntVector GridDimensions;
	float      CellSize;

	FVector    AmbientWind;
	FMicroburstConfig BurstCfg;
	float      CurrentTime;
	float      TurbulencePhase;

	void ResizeGrid(const FBox& Bounds, float CellSizeMeters);
	int32 GetCellIndex(int32 X, int32 Y, int32 Z) const;
	FIntVector WorldToCell(const FVector& Pos) const;
	FVector    CellToWorld(int32 X, int32 Y, int32 Z) const;

	FVector ComputeMicroburstVelocity(const FVector& Pos, float SimTime) const;
	FVector ComputeTurbulence(const FVector& Pos, float SimTime) const;
	float   MicroburstTimeEnvelope(float SimTime) const;

	void UpdateGridFromAnalytic(float SimTime);

	void ClearGrid();
};
