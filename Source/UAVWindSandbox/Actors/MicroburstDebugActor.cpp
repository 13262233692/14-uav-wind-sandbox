#include "Actors/MicroburstDebugActor.h"
#include "DrawDebugHelpers.h"

AMicroburstDebugActor::AMicroburstDebugActor()
{
	PrimaryActorTick.bCanEverTick = true;
	SimTime = 0.0f;
	bActive = false;
}

void AMicroburstDebugActor::BeginPlay()
{
	Super::BeginPlay();
	WindField.Initialize(FieldBounds, CellSize);
	WindField.SetMicroburstConfig(BurstConfig);
	WindField.SetAmbientWind(AmbientWind);
}

void AMicroburstDebugActor::TriggerMicroburstAt(FVector Center, float IntensityScale)
{
	FMicroburstConfig Cfg = BurstConfig;
	Cfg.CenterPosition = Center;
	Cfg.ShearIntensity = IntensityScale;
	Cfg.StartTime = SimTime;
	Cfg.PeakTime = SimTime + Cfg.EvolutionTime * 0.6f;
	WindField.SetMicroburstConfig(Cfg);
	bActive = true;
}

void AMicroburstDebugActor::DissipateMicroburst()
{
	FMicroburstConfig Cfg = BurstConfig;
	Cfg.ShearIntensity = 0.0f;
	WindField.SetMicroburstConfig(Cfg);
	bActive = false;
}

float AMicroburstDebugActor::GetCurrentIntensity() const
{
	return WindField.GetMicroburstIntensityAtTime(SimTime);
}

FVector AMicroburstDebugActor::GetWindAt(FVector WorldPos) const
{
	return WindField.SampleWindVelocity(WorldPos);
}

void AMicroburstDebugActor::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	SimTime += DeltaSeconds;
	WindField.UpdateField(SimTime, DeltaSeconds);

	if (!bDrawVelocityField && !bDrawDowndraftCore && !bDrawOutflowRing) return;

	float Intensity = WindField.GetMicroburstIntensityAtTime(SimTime);
	if (Intensity < 0.01f) return;

	FVector Center = WindField.GetMicroburstConfig().CenterPosition;

	if (bDrawDowndraftCore)
	{
		float CoreR = WindField.GetMicroburstConfig().CoreRadius;
		float HTop  = WindField.GetMicroburstConfig().HeightTop;
		float HBot  = WindField.GetMicroburstConfig().HeightBottom;

		FColor CoreColor = FColor::Red;
		float Alpha = FMath::Lerp(0.3f, 1.0f, Intensity);

		DrawDebugCylinder(GetWorld(),
			Center + FVector(0, 0, HBot),
			Center + FVector(0, 0, HTop),
			CoreR, 16, CoreColor, false, 0.0f, 0, 2.0f);

		DrawDebugCircle(GetWorld(),
			Center + FVector(0, 0, HBot),
			CoreR, 32, CoreColor, false, 0.0f, 0, FVector(1, 0, 0), FVector(0, 1, 0), 2.0f);
	}

	if (bDrawOutflowRing)
	{
		float OutR = WindField.GetMicroburstConfig().OutflowRadius;
		float HBot = WindField.GetMicroburstConfig().HeightBottom;

		FColor RingColor(255, 165, 0);

		DrawDebugCircle(GetWorld(),
			Center + FVector(0, 0, HBot + 50.0f),
			OutR, 48, RingColor, false, 0.0f, 0, FVector(1, 0, 0), FVector(0, 1, 0), 2.0f);
	}

	if (bDrawVelocityField)
	{
		FVector Origin = GetActorLocation();
		float Spacing = SampleSpacing;
		float Scale = ArrowLengthScale;

		int32 Count = 5;
		for (int32 DX = -Count; DX <= Count; DX++)
		{
			for (int32 DY = -Count; DY <= Count; DY++)
			{
				for (int32 DZ = -2; DZ <= 2; DZ++)
				{
					FVector SamplePos = Origin + FVector(
						(float)DX * Spacing,
						(float)DY * Spacing,
						(float)DZ * Spacing
					);

					FVector Wind = WindField.SampleWindVelocity(SamplePos);
					float Mag = Wind.Size();
					if (Mag < 0.5f) continue;

					FColor ArrowCol;
					if (Wind.Z < -3.0f)
						ArrowCol = FColor::Red;
					else if (Mag > 20.0f)
						ArrowCol = FColor::Yellow;
					else
						ArrowCol = FColor::Green;

					FVector End = SamplePos + Wind.GetSafeNormal() * FMath::Min(Mag * Scale, 250.0f);
					DrawDebugDirectionalArrow(GetWorld(), SamplePos, End, 30.0f, ArrowCol, false, 0.0f, 0, 1.5f);
				}
			}
		}
	}
}
