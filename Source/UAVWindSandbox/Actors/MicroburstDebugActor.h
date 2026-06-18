#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Physics/MicroburstWindField.h"
#include "MicroburstDebugActor.generated.h"

UCLASS()
class UAVWINDSANDBOX_API AMicroburstDebugActor : public AActor
{
	GENERATED_BODY()

public:
	AMicroburstDebugActor();

	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="WindDebug")
	FMicroburstConfig BurstConfig;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="WindDebug")
	FBox FieldBounds = FBox(FVector(-2000, -2000, 0), FVector(2000, 2000, 1500));

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="WindDebug")
	float CellSize = 100.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="WindDebug")
	float SampleSpacing = 300.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="WindDebug")
	float ArrowLengthScale = 5.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="WindDebug")
	bool bDrawVelocityField = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="WindDebug")
	bool bDrawDowndraftCore = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="WindDebug")
	bool bDrawOutflowRing = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="WindDebug")
	FVector AmbientWind = FVector(5.0f, 0.0f, 0.0f);

	UFUNCTION(BlueprintCallable, Category="WindDebug")
	void TriggerMicroburstAt(FVector Center, float IntensityScale);

	UFUNCTION(BlueprintCallable, Category="WindDebug")
	void DissipateMicroburst();

	UFUNCTION(BlueprintPure, Category="WindDebug")
	float GetCurrentIntensity() const;

	UFUNCTION(BlueprintPure, Category="WindDebug")
	FVector GetWindAt(FVector WorldPos) const;

private:
	FMicroburstWindField WindField;
	float SimTime;
	bool bActive;
};

