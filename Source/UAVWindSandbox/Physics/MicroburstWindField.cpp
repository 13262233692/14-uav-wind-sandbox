#include "Physics/MicroburstWindField.h"
#include <cmath>

FMicroburstWindField::FMicroburstWindField()
	: GridMin(0.0f, 0.0f, 0.0f)
	, GridMax(0.0f, 0.0f, 0.0f)
	, GridDimensions(0, 0, 0)
	, CellSize(1.0f)
	, AmbientWind(5.0f, 0.0f, 0.0f)
	, CurrentTime(0.0f)
	, TurbulencePhase(0.0f)
{
}

void FMicroburstWindField::Initialize(const FBox& WorldBounds, float CellSizeMeters)
{
	ResizeGrid(WorldBounds, CellSizeMeters);
	ClearGrid();
}

void FMicroburstWindField::SetMicroburstConfig(const FMicroburstConfig& Config)
{
	BurstCfg = Config;
}

void FMicroburstWindField::ResizeGrid(const FBox& Bounds, float CellSizeMeters)
{
	CellSize = FMath::Max(CellSizeMeters, 1.0f);
	GridMin = Bounds.Min;
	GridMax = Bounds.Max;

	FVector Size = Bounds.GetSize();
	GridDimensions.X = FMath::Max(1, FMath::CeilToInt(Size.X / CellSize));
	GridDimensions.Y = FMath::Max(1, FMath::CeilToInt(Size.Y / CellSize));
	GridDimensions.Z = FMath::Max(1, FMath::CeilToInt(Size.Z / CellSize));

	int32 Total = GridDimensions.X * GridDimensions.Y * GridDimensions.Z;
	Grid.SetNum(Total);
}

void FMicroburstWindField::ClearGrid()
{
	for (FWindCell& Cell : Grid)
	{
		Cell.Velocity = AmbientWind;
		Cell.Density  = 1.225f;
		Cell.Pressure = 101325.0f;
	}
}

int32 FMicroburstWindField::GetCellIndex(int32 X, int32 Y, int32 Z) const
{
	return Z * GridDimensions.X * GridDimensions.Y + Y * GridDimensions.X + X;
}

FIntVector FMicroburstWindField::WorldToCell(const FVector& Pos) const
{
	FVector Local = Pos - GridMin;
	return FIntVector(
		FMath::Clamp(FMath::FloorToInt(Local.X / CellSize), 0, GridDimensions.X - 1),
		FMath::Clamp(FMath::FloorToInt(Local.Y / CellSize), 0, GridDimensions.Y - 1),
		FMath::Clamp(FMath::FloorToInt(Local.Z / CellSize), 0, GridDimensions.Z - 1)
	);
}

FVector FMicroburstWindField::CellToWorld(int32 X, int32 Y, int32 Z) const
{
	return GridMin + FVector(
		(float)X * CellSize + CellSize * 0.5f,
		(float)Y * CellSize + CellSize * 0.5f,
		(float)Z * CellSize + CellSize * 0.5f
	);
}

float FMicroburstWindField::MicroburstTimeEnvelope(float SimTime) const
{
	if (SimTime < BurstCfg.StartTime) return 0.0f;
	if (SimTime > BurstCfg.StartTime + BurstCfg.EvolutionTime) return 0.0f;

	float T = (SimTime - BurstCfg.StartTime) / BurstCfg.EvolutionTime;

	float Rise = FMath::Clamp(T / (BurstCfg.PeakTime / BurstCfg.EvolutionTime), 0.0f, 1.0f);
	float Fall = FMath::Clamp((1.0f - T) / (1.0f - BurstCfg.PeakTime / BurstCfg.EvolutionTime), 0.0f, 1.0f);

	return FMath::Min(Rise, Fall);
}

float FMicroburstWindField::GetMicroburstIntensityAtTime(float SimTime) const
{
	return MicroburstTimeEnvelope(SimTime);
}

FVector FMicroburstWindField::ComputeMicroburstVelocity(const FVector& Pos, float SimTime) const
{
	float Envelope = MicroburstTimeEnvelope(SimTime);
	if (Envelope < 0.001f) return FVector::ZeroVector;

	FVector Rel = Pos - BurstCfg.CenterPosition;

	float HeightRatio = 1.0f;
	float HeightSpan = BurstCfg.HeightTop - BurstCfg.HeightBottom;
	if (HeightSpan > 0.0f)
	{
		float NormZ = (Pos.Z - BurstCfg.CenterPosition.Z + HeightSpan * 0.5f) / HeightSpan;
		HeightRatio = FMath::Sin(FMath::Clamp(NormZ, 0.0f, 1.0f) * PI);
	}

	float HorizontalDist = FMath::Sqrt(Rel.X * Rel.X + Rel.Y * Rel.Y);
	float RadialFactor = 1.0f;

	if (HorizontalDist < BurstCfg.OutflowRadius)
	{
		float NormR = HorizontalDist / BurstCfg.OutflowRadius;

		float Rcore = BurstCfg.CoreRadius / BurstCfg.OutflowRadius;

		if (NormR < Rcore)
		{
			RadialFactor = NormR / Rcore;
		}
		else
		{
			float T = (NormR - Rcore) / (1.0f - Rcore);
			RadialFactor = FMath::Cos(T * PI * 0.5f);
		}
	}
	else
	{
		return FVector::ZeroVector;
	}

	float DowndraftMag = BurstCfg.MaxDowndraftSpeed * Envelope * HeightRatio * RadialFactor;

	float OutflowMag = 0.0f;
	float ZNorm = 1.0f;
	if (HeightSpan > 0.0f)
	{
		float ZDist = Pos.Z - (BurstCfg.CenterPosition.Z - HeightSpan * 0.3f);
		ZNorm = FMath::Exp(-FMath::Square(ZDist / (HeightSpan * 0.4f)));
	}

	if (HorizontalDist < BurstCfg.CoreRadius)
	{
		float NormR = HorizontalDist / BurstCfg.CoreRadius;
		OutflowMag = BurstCfg.MaxOutflowSpeed * Envelope * ZNorm * NormR;
	}
	else if (HorizontalDist < BurstCfg.OutflowRadius)
	{
		float NormR = (HorizontalDist - BurstCfg.CoreRadius)
			/ (BurstCfg.OutflowRadius - BurstCfg.CoreRadius);
		OutflowMag = BurstCfg.MaxOutflowSpeed * Envelope * ZNorm * FMath::Exp(-NormR * 3.0f);
	}

	FVector OutflowDir(0.0f, 0.0f, 0.0f);
	if (HorizontalDist > 0.1f)
	{
		OutflowDir = FVector(Rel.X / HorizontalDist, Rel.Y / HorizontalDist, 0.0f);
	}

	FVector Wind(
		OutflowDir.X * OutflowMag,
		OutflowDir.Y * OutflowMag,
		-DowndraftMag
	);

	return Wind * BurstCfg.ShearIntensity;
}

FVector FMicroburstWindField::ComputeTurbulence(const FVector& Pos, float SimTime) const
{
	float Envelope = MicroburstTimeEnvelope(SimTime);
	if (Envelope < 0.001f) return FVector::ZeroVector;

	FVector Rel = Pos - BurstCfg.CenterPosition;
	float HorizontalDist = FMath::Sqrt(Rel.X * Rel.X + Rel.Y * Rel.Y);

	if (HorizontalDist > BurstCfg.OutflowRadius * 1.2f) return FVector::ZeroVector;

	float TurbScale = 50.0f;
	float Gain = BurstCfg.TurbulenceGain * BurstCfg.MaxOutflowSpeed * Envelope;

	float Nx = FMath::Sin((Pos.X / TurbScale) + SimTime * 1.3f)
			 + FMath::Sin((Pos.Y / (TurbScale * 0.7f)) + SimTime * 1.7f)
			 + FMath::Sin((Pos.Z / (TurbScale * 1.2f)) + SimTime * 2.1f);

	float Ny = FMath::Sin((Pos.Y / TurbScale) + SimTime * 1.1f)
			 + FMath::Sin((Pos.Z / (TurbScale * 0.8f)) + SimTime * 1.9f)
			 + FMath::Sin((Pos.X / (TurbScale * 1.3f)) + SimTime * 2.3f);

	float Nz = FMath::Sin((Pos.Z / TurbScale) + SimTime * 0.9f)
			 + FMath::Sin((Pos.X / (TurbScale * 0.6f)) + SimTime * 1.5f)
			 + FMath::Sin((Pos.Y / (TurbScale * 1.1f)) + SimTime * 2.0f);

	float RadialFade = FMath::Max(0.0f, 1.0f - HorizontalDist / (BurstCfg.OutflowRadius * 1.2f));

	return FVector(Nx, Ny, Nz) * Gain * RadialFade / 3.0f;
}

bool FMicroburstWindField::IsInsideMicroburst(const FVector& WorldPosition) const
{
	FVector Rel = WorldPosition - BurstCfg.CenterPosition;
	float HDist = FMath::Sqrt(Rel.X * Rel.X + Rel.Y * Rel.Y);
	float HeightSpan = BurstCfg.HeightTop - BurstCfg.HeightBottom;

	return HDist < BurstCfg.OutflowRadius * 1.1f
		&& FMath::Abs(Rel.Z) < HeightSpan * 0.6f;
}

void FMicroburstWindField::UpdateGridFromAnalytic(float SimTime)
{
	for (int32 Z = 0; Z < GridDimensions.Z; ++Z)
	{
		for (int32 Y = 0; Y < GridDimensions.Y; ++Y)
		{
			for (int32 X = 0; X < GridDimensions.X; ++X)
			{
				FVector WorldPos = CellToWorld(X, Y, Z);
				int32 Idx = GetCellIndex(X, Y, Z);

				FVector Micro = ComputeMicroburstVelocity(WorldPos, SimTime);
				FVector Turb  = ComputeTurbulence(WorldPos, SimTime);

				Grid[Idx].Velocity = AmbientWind + Micro + Turb;

				float AltitudeFactor = FMath::Exp(-WorldPos.Z / 8000.0f);
				Grid[Idx].Density = 1.225f * AltitudeFactor;
			}
		}
	}
}

void FMicroburstWindField::UpdateField(float SimTime, float DeltaTime)
{
	CurrentTime = SimTime;
	TurbulencePhase += DeltaTime * 1.5f;

	if (Grid.Num() > 0)
	{
		UpdateGridFromAnalytic(SimTime);
	}
}

FVector FMicroburstWindField::SampleWindVelocity(const FVector& WorldPosition) const
{
	FVector Micro = ComputeMicroburstVelocity(WorldPosition, CurrentTime);
	FVector Turb  = ComputeTurbulence(WorldPosition, CurrentTime);
	return AmbientWind + Micro + Turb;
}

float FMicroburstWindField::SampleAirDensity(const FVector& WorldPosition) const
{
	float AltitudeFactor = FMath::Exp(-WorldPosition.Z / 8000.0f);
	return 1.225f * AltitudeFactor;
}

FVector FMicroburstWindField::SampleWindShear(const FVector& WorldPosition) const
{
	float DeltaH = 10.0f;
	FVector WindBelow = SampleWindVelocity(WorldPosition + FVector(0, 0, -DeltaH));
	FVector WindAbove = SampleWindVelocity(WorldPosition + FVector(0, 0, DeltaH));
	return (WindAbove - WindBelow) / (2.0f * DeltaH);
}
