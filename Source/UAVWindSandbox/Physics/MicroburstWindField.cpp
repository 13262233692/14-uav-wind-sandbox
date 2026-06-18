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

void FMicroburstWindField::ResetHealth()
{
	Health = EWindFieldHealth::Healthy;
	TotalClamps = 0;
}

void FMicroburstWindField::MarkHealth(EWindFieldHealth NewHealth) const
{
	if (NewHealth > Health)
	{
		Health = NewHealth;
		if (NewHealth >= EWindFieldHealth::OutputClamped)
		{
			TotalClamps++;
		}
	}
}

bool FMicroburstWindField::ValidateVector(const FVector& V) const
{
	return FPlatformMath::IsFinite(V.X)
		&& FPlatformMath::IsFinite(V.Y)
		&& FPlatformMath::IsFinite(V.Z);
}

bool FMicroburstWindField::ValidateFloat(float Val) const
{
	return FPlatformMath::IsFinite(Val);
}

FVector FMicroburstWindField::ClampVelocity(const FVector& Velocity) const
{
	FVector Result = Velocity;
	if (!ValidateVector(Result))
	{
		MarkHealth(EWindFieldHealth::OutputClamped);
		return FVector::ZeroVector;
	}

	float Mag = Result.Size();
	if (Mag > UAV_WIND_MAX_SPEED)
	{
		Result = Result.GetSafeNormal() * UAV_WIND_MAX_SPEED;
		MarkHealth(EWindFieldHealth::OutputClamped);
	}
	return Result;
}

float FMicroburstWindField::SafeSqrt(float Val) const
{
	if (!ValidateFloat(Val) || Val < 0.0f)
	{
		MarkHealth(EWindFieldHealth::InputClamped);
		return 0.0f;
	}
	return FMath::Sqrt(Val);
}

float FMicroburstWindField::SafeDivide(float Numerator, float Denominator, float Default) const
{
	float AbsDenom = FMath::Abs(Denominator);
	if (!ValidateFloat(Numerator) || !ValidateFloat(Denominator) || AbsDenom < UAV_WIND_EPSILON)
	{
		MarkHealth(EWindFieldHealth::InputClamped);
		return Default;
	}
	float Result = Numerator / Denominator;
	if (!ValidateFloat(Result))
	{
		MarkHealth(EWindFieldHealth::OutputClamped);
		return Default;
	}
	return Result;
}

float FMicroburstWindField::SafeSin(float Val) const
{
	if (!ValidateFloat(Val))
	{
		MarkHealth(EWindFieldHealth::InputClamped);
		return 0.0f;
	}
	return FMath::Sin(Val);
}

float FMicroburstWindField::SafeCos(float Val) const
{
	if (!ValidateFloat(Val))
	{
		MarkHealth(EWindFieldHealth::InputClamped);
		return 1.0f;
	}
	return FMath::Cos(Val);
}

float FMicroburstWindField::SafeExp(float Val) const
{
	if (!ValidateFloat(Val))
	{
		MarkHealth(EWindFieldHealth::InputClamped);
		return 1.0f;
	}
	float ClampedVal = FMath::Clamp(Val, -50.0f, 50.0f);
	if (Val != ClampedVal)
	{
		MarkHealth(EWindFieldHealth::InputClamped);
	}
	return FMath::Exp(ClampedVal);
}

void FMicroburstWindField::Initialize(const FBox& WorldBounds, float CellSizeMeters)
{
	ResizeGrid(WorldBounds, CellSizeMeters);
	ClearGrid();
	ResetHealth();
}

void FMicroburstWindField::SetMicroburstConfig(const FMicroburstConfig& Config)
{
	FMicroburstConfig SafeCfg = Config;

	if (!ValidateVector(SafeCfg.CenterPosition))
	{
		SafeCfg.CenterPosition = FVector(0, 0, 500);
		MarkHealth(EWindFieldHealth::InputClamped);
	}

	SafeCfg.MaxDowndraftSpeed = FMath::Clamp(SafeCfg.MaxDowndraftSpeed, 0.0f, UAV_WIND_MAX_SPEED);
	SafeCfg.MaxOutflowSpeed   = FMath::Clamp(SafeCfg.MaxOutflowSpeed,   0.0f, UAV_WIND_MAX_SPEED);
	SafeCfg.CoreRadius        = FMath::Max(SafeCfg.CoreRadius, 10.0f);
	SafeCfg.OutflowRadius     = FMath::Max(SafeCfg.OutflowRadius, SafeCfg.CoreRadius + 10.0f);
	SafeCfg.HeightTop         = FMath::Max(SafeCfg.HeightTop, 100.0f);
	SafeCfg.HeightBottom      = FMath::Clamp(SafeCfg.HeightBottom, 0.0f, SafeCfg.HeightTop - 10.0f);
	SafeCfg.EvolutionTime     = FMath::Max(SafeCfg.EvolutionTime, 10.0f);
	SafeCfg.StartTime         = FMath::Max(SafeCfg.StartTime, 0.0f);
	SafeCfg.PeakTime          = FMath::Clamp(SafeCfg.PeakTime, SafeCfg.StartTime + 5.0f, SafeCfg.StartTime + SafeCfg.EvolutionTime - 5.0f);
	SafeCfg.ShearIntensity    = FMath::Clamp(SafeCfg.ShearIntensity, 0.0f, 3.0f);
	SafeCfg.TurbulenceGain    = FMath::Clamp(SafeCfg.TurbulenceGain, 0.0f, 1.0f);

	BurstCfg = SafeCfg;
}

void FMicroburstWindField::SetAmbientWind(const FVector& WindVel)
{
	AmbientWind = ClampVelocity(WindVel);
}

void FMicroburstWindField::ResizeGrid(const FBox& Bounds, float CellSizeMeters)
{
	CellSize = FMath::Max(CellSizeMeters, 1.0f);
	GridMin = Bounds.Min;
	GridMax = Bounds.Max;

	if (!ValidateVector(GridMin) || !ValidateVector(GridMax))
	{
		GridMin = FVector(-2000, -2000, 0);
		GridMax = FVector(2000, 2000, 1500);
		MarkHealth(EWindFieldHealth::InputClamped);
	}

	FVector Size = GridMax - GridMin;
	GridDimensions.X = FMath::Max(1, FMath::Clamp(FMath::CeilToInt(Size.X / CellSize), 1, 1000));
	GridDimensions.Y = FMath::Max(1, FMath::Clamp(FMath::CeilToInt(Size.Y / CellSize), 1, 1000));
	GridDimensions.Z = FMath::Max(1, FMath::Clamp(FMath::CeilToInt(Size.Z / CellSize), 1, 1000));

	int32 Total = GridDimensions.X * GridDimensions.Y * GridDimensions.Z;
	Total = FMath::Clamp(Total, 1, 10000000);
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
	if (!ValidateFloat(SimTime))
	{
		MarkHealth(EWindFieldHealth::InputClamped);
		return 0.0f;
	}

	if (SimTime < BurstCfg.StartTime) return 0.0f;
	if (SimTime > BurstCfg.StartTime + BurstCfg.EvolutionTime) return 0.0f;

	float T = SafeDivide(SimTime - BurstCfg.StartTime, BurstCfg.EvolutionTime, 0.0f);
	float PeakT = SafeDivide(BurstCfg.PeakTime - BurstCfg.StartTime, BurstCfg.EvolutionTime, 0.5f);

	float Rise = FMath::Clamp(SafeDivide(T, PeakT, 1.0f), 0.0f, 1.0f);
	float Fall = FMath::Clamp(SafeDivide(1.0f - T, 1.0f - PeakT, 1.0f), 0.0f, 1.0f);

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

	if (!ValidateVector(Pos))
	{
		MarkHealth(EWindFieldHealth::InputClamped);
		return FVector::ZeroVector;
	}

	FVector Rel = Pos - BurstCfg.CenterPosition;

	float HeightSpan = FMath::Max(BurstCfg.HeightTop - BurstCfg.HeightBottom, 10.0f);
	float HeightRatio = 1.0f;

	if (HeightSpan > 0.0f)
	{
		float NormZ = SafeDivide(Pos.Z - BurstCfg.CenterPosition.Z + HeightSpan * 0.5f, HeightSpan, 0.5f);
		NormZ = FMath::Clamp(NormZ, 0.0f, 1.0f);
		HeightRatio = SafeSin(NormZ * PI);
	}

	float RelXSq = Rel.X * Rel.X;
	float RelYSq = Rel.Y * Rel.Y;
	float HorizontalDist = SafeSqrt(RelXSq + RelYSq);

	float RadialFactor = 1.0f;

	if (HorizontalDist < BurstCfg.OutflowRadius)
	{
		float NormR = SafeDivide(HorizontalDist, BurstCfg.OutflowRadius, 0.0f);
		float Rcore = SafeDivide(BurstCfg.CoreRadius, BurstCfg.OutflowRadius, 0.3f);

		if (NormR < Rcore)
		{
			RadialFactor = SafeDivide(NormR, Rcore, 0.0f);
		}
		else
		{
			float T = SafeDivide(NormR - Rcore, 1.0f - Rcore, 0.0f);
			RadialFactor = SafeCos(T * PI * 0.5f);
		}
	}
	else
	{
		return FVector::ZeroVector;
	}

	float DowndraftMag = BurstCfg.MaxDowndraftSpeed * Envelope * HeightRatio * RadialFactor;
	DowndraftMag = FMath::Clamp(DowndraftMag, 0.0f, UAV_WIND_MAX_SPEED);

	float OutflowMag = 0.0f;
	float ZNorm = 1.0f;

	if (HeightSpan > 0.0f)
	{
		float ZDist = Pos.Z - (BurstCfg.CenterPosition.Z - HeightSpan * 0.3f);
		float ExpArg = -FMath::Square(SafeDivide(ZDist, HeightSpan * 0.4f, 1.0f));
		ZNorm = SafeExp(ExpArg);
	}

	if (HorizontalDist < BurstCfg.CoreRadius)
	{
		float NormR = SafeDivide(HorizontalDist, BurstCfg.CoreRadius, 0.0f);
		OutflowMag = BurstCfg.MaxOutflowSpeed * Envelope * ZNorm * NormR;
	}
	else if (HorizontalDist < BurstCfg.OutflowRadius)
	{
		float NormR = SafeDivide(HorizontalDist - BurstCfg.CoreRadius,
			BurstCfg.OutflowRadius - BurstCfg.CoreRadius, 0.0f);
		float ExpArg = -NormR * 3.0f;
		OutflowMag = BurstCfg.MaxOutflowSpeed * Envelope * ZNorm * SafeExp(ExpArg);
	}

	OutflowMag = FMath::Clamp(OutflowMag, 0.0f, UAV_WIND_MAX_SPEED);

	FVector OutflowDir(0.0f, 0.0f, 0.0f);
	if (HorizontalDist > 0.1f)
	{
		OutflowDir = FVector(
			SafeDivide(Rel.X, HorizontalDist, 0.0f),
			SafeDivide(Rel.Y, HorizontalDist, 0.0f),
			0.0f
		);
	}

	FVector Wind(
		OutflowDir.X * OutflowMag,
		OutflowDir.Y * OutflowMag,
		-DowndraftMag
	);

	return ClampVelocity(Wind * BurstCfg.ShearIntensity);
}

FVector FMicroburstWindField::ComputeTurbulence(const FVector& Pos, float SimTime) const
{
	float Envelope = MicroburstTimeEnvelope(SimTime);
	if (Envelope < 0.001f) return FVector::ZeroVector;

	if (!ValidateVector(Pos))
	{
		MarkHealth(EWindFieldHealth::InputClamped);
		return FVector::ZeroVector;
	}

	FVector Rel = Pos - BurstCfg.CenterPosition;
	float RelXSq = Rel.X * Rel.X;
	float RelYSq = Rel.Y * Rel.Y;
	float HorizontalDist = SafeSqrt(RelXSq + RelYSq);

	if (HorizontalDist > BurstCfg.OutflowRadius * 1.2f) return FVector::ZeroVector;

	float TurbScale = 50.0f;
	float Gain = BurstCfg.TurbulenceGain * BurstCfg.MaxOutflowSpeed * Envelope;
	Gain = FMath::Min(Gain, UAV_WIND_MAX_SPEED * 0.3f);

	float Nx = SafeSin(SafeDivide(Pos.X, TurbScale, 0.0f) + SimTime * 1.3f)
			 + SafeSin(SafeDivide(Pos.Y, TurbScale * 0.7f, 0.0f) + SimTime * 1.7f)
			 + SafeSin(SafeDivide(Pos.Z, TurbScale * 1.2f, 0.0f) + SimTime * 2.1f);

	float Ny = SafeSin(SafeDivide(Pos.Y, TurbScale, 0.0f) + SimTime * 1.1f)
			 + SafeSin(SafeDivide(Pos.Z, TurbScale * 0.8f, 0.0f) + SimTime * 1.9f)
			 + SafeSin(SafeDivide(Pos.X, TurbScale * 1.3f, 0.0f) + SimTime * 2.3f);

	float Nz = SafeSin(SafeDivide(Pos.Z, TurbScale, 0.0f) + SimTime * 0.9f)
			 + SafeSin(SafeDivide(Pos.X, TurbScale * 0.6f, 0.0f) + SimTime * 1.5f)
			 + SafeSin(SafeDivide(Pos.Y, TurbScale * 1.1f, 0.0f) + SimTime * 2.0f);

	float RadialFade = FMath::Max(0.0f, 1.0f - SafeDivide(HorizontalDist, BurstCfg.OutflowRadius * 1.2f, 0.0f));

	FVector Turb(Nx, Ny, Nz);
	Turb = ClampVelocity(Turb * Gain * RadialFade / 3.0f);

	return Turb;
}

bool FMicroburstWindField::IsInsideMicroburst(const FVector& WorldPosition) const
{
	if (!ValidateVector(WorldPosition)) return false;

	FVector Rel = WorldPosition - BurstCfg.CenterPosition;
	float RelXSq = Rel.X * Rel.X;
	float RelYSq = Rel.Y * Rel.Y;
	float HDist = SafeSqrt(RelXSq + RelYSq);
	float HeightSpan = FMath::Max(BurstCfg.HeightTop - BurstCfg.HeightBottom, 10.0f);

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

				Grid[Idx].Velocity = ClampVelocity(AmbientWind + Micro + Turb);

				float AltitudeFactor = SafeExp(-SafeDivide(WorldPos.Z, 8000.0f, 0.0f));
				Grid[Idx].Density = FMath::Clamp(1.225f * AltitudeFactor,
					UAV_WIND_MIN_DENSITY, UAV_WIND_MAX_DENSITY);
			}
		}
	}
}

void FMicroburstWindField::UpdateField(float SimTime, float DeltaTime)
{
	CurrentTime = FMath::Max(0.0f, SimTime);
	if (!ValidateFloat(DeltaTime) || DeltaTime <= 0.0f)
	{
		DeltaTime = 0.0167f;
		MarkHealth(EWindFieldHealth::InputClamped);
	}
	TurbulencePhase += DeltaTime * 1.5f;

	if (Grid.Num() > 0)
	{
		UpdateGridFromAnalytic(SimTime);
	}
}

FVector FMicroburstWindField::SampleWindVelocity(const FVector& WorldPosition) const
{
	if (!ValidateVector(WorldPosition))
	{
		MarkHealth(EWindFieldHealth::InputClamped);
		return AmbientWind;
	}

	FVector Micro = ComputeMicroburstVelocity(WorldPosition, CurrentTime);
	FVector Turb  = ComputeTurbulence(WorldPosition, CurrentTime);
	return ClampVelocity(AmbientWind + Micro + Turb);
}

float FMicroburstWindField::SampleAirDensity(const FVector& WorldPosition) const
{
	if (!ValidateVector(WorldPosition))
	{
		MarkHealth(EWindFieldHealth::InputClamped);
		return 1.225f;
	}

	float AltitudeFactor = SafeExp(-SafeDivide(WorldPos.Z, 8000.0f, 0.0f));
	float Density = 1.225f * AltitudeFactor;
	return FMath::Clamp(Density, UAV_WIND_MIN_DENSITY, UAV_WIND_MAX_DENSITY);
}

FVector FMicroburstWindField::SampleWindShear(const FVector& WorldPosition) const
{
	if (!ValidateVector(WorldPosition))
	{
		MarkHealth(EWindFieldHealth::InputClamped);
		return FVector::ZeroVector;
	}

	float DeltaH = FMath::Max(10.0f, CellSize);
	FVector WindBelow = SampleWindVelocity(WorldPosition + FVector(0, 0, -DeltaH));
	FVector WindAbove = SampleWindVelocity(WorldPosition + FVector(0, 0, DeltaH));

	float Denom = 2.0f * DeltaH;
	FVector Shear = FVector(
		SafeDivide(WindAbove.X - WindBelow.X, Denom, 0.0f),
		SafeDivide(WindAbove.Y - WindBelow.Y, Denom, 0.0f),
		SafeDivide(WindAbove.Z - WindBelow.Z, Denom, 0.0f)
	);

	Shear.X = FMath::Clamp(Shear.X, -UAV_WIND_MAX_SHEAR, UAV_WIND_MAX_SHEAR);
	Shear.Y = FMath::Clamp(Shear.Y, -UAV_WIND_MAX_SHEAR, UAV_WIND_MAX_SHEAR);
	Shear.Z = FMath::Clamp(Shear.Z, -UAV_WIND_MAX_SHEAR, UAV_WIND_MAX_SHEAR);

	if (!ValidateVector(Shear))
	{
		MarkHealth(EWindFieldHealth::OutputClamped);
		return FVector::ZeroVector;
	}

	return Shear;
}
