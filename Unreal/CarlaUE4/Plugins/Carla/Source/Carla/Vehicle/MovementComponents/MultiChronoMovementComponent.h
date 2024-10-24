// MultiChronoMovementComponent.h

#pragma once

#include "BaseCarlaMovementComponent.h"
#include "Carla/Vehicle/VehicleControl.h"

#ifdef WITH_CHRONO
#include "compiler/disable-ue4-macros.h"

#if defined(__clang__)
#  pragma clang diagnostic push
#  pragma clang diagnostic ignored "-Wshadow"
#endif

#include "chrono/physics/ChSystemNSC.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/ChTerrain.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#if defined(__clang__)
#  pragma clang diagnostic pop
#endif

#include "compiler/enable-ue4-macros.h"
#endif

#include "MultiChronoMovementComponent.generated.h"

#ifdef WITH_CHRONO
class UERayCastTerrainMulti : public chrono::vehicle::ChTerrain
{
public:
  UERayCastTerrainMulti();

  virtual double GetHeight(const chrono::ChVector<>& loc) const override;
  virtual chrono::ChVector<> GetNormal(const chrono::ChVector<>& loc) const override;
  virtual float GetCoefficientFriction(const chrono::ChVector<>& loc) const override;
};
#endif

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class CARLA_API UMultiChronoMovementComponent : public UBaseCarlaMovementComponent
{
  GENERATED_BODY()

public:
  UMultiChronoMovementComponent();

  static void CreateMultiChronoMovementComponent(
      const TArray<ACarlaWheeledVehicle*>& Vehicles,
      uint64_t MaxSubsteps,
      float MaxSubstepDeltaTime,
      const FString& VehicleJSON = "",
      const FString& PowertrainJSON = "",
      const FString& TireJSON = "",
      const FString& BaseJSONPath = "");

#ifdef WITH_CHRONO
  virtual void BeginPlay() override;

  void InitializeChronoVehicles();

  void ProcessControl(FVehicleControl& Control) override;

  virtual void TickComponent(
      float DeltaTime,
      ELevelTick TickType,
      FActorComponentTickFunction* ThisTickFunction) override;

  void AdvanceChronoSimulation(float StepSize);

  virtual FVector GetVelocity() const override;

  virtual int32 GetVehicleCurrentGear() const override;

  virtual float GetVehicleForwardSpeed() const override;

  virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
#endif

private:
#ifdef WITH_CHRONO
  chrono::ChSystemNSC Sys;

  TArray<ACarlaWheeledVehicle*> CarlaVehicles;

  // Maps to store Chrono vehicles corresponding to Carla vehicles
  TMap<ACarlaWheeledVehicle*, std::shared_ptr<chrono::vehicle::WheeledVehicle>> ChronoVehicles;

  std::shared_ptr<UERayCastTerrainMulti> Terrain;

  uint64_t MaxSubsteps = 10;
  float MaxSubstepDeltaTime = 0.01f;
  FVehicleControl VehicleControl;

  FString VehicleJSON = "hmmwv/vehicle/HMMWV_Vehicle.json";
  FString PowertrainJSON = "hmmwv/powertrain/HMMWV_ShaftsPowertrain.json";
  FString TireJSON = "hmmwv/tire/HMMWV_Pac02Tire.json";
  FString BaseJSONPath = "";

  void DisableChronoPhysics();

  UFUNCTION()
  void OnVehicleHit(AActor* Actor, AActor* OtherActor, FVector NormalImpulse, const FHitResult& Hit);

  UFUNCTION()
  void OnVehicleOverlap(
      UPrimitiveComponent* OverlappedComponent,
      AActor* OtherActor,
      UPrimitiveComponent* OtherComp,
      int32 OtherBodyIndex,
      bool bFromSweep,
      const FHitResult& SweepResult);
#endif
};