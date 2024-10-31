// Copyright (c) 2021 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
// Copyright (c) 2019 Intel Corporation
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

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

#include "ChronoMovementComponent.generated.h"

#ifdef WITH_CHRONO
class UERayCastTerrain : public chrono::vehicle::ChTerrain
{
  ACarlaWheeledVehicle* CarlaVehicle;
  chrono::vehicle::ChVehicle* ChronoVehicle;

public:
  UERayCastTerrain(ACarlaWheeledVehicle* UEVehicle, chrono::vehicle::ChVehicle* ChrVehicle);

  std::pair<bool, FHitResult> GetTerrainProperties(const FVector &Location) const;
  virtual double GetHeight(const chrono::ChVector<>& loc) const override;
  virtual chrono::ChVector<> GetNormal(const chrono::ChVector<>& loc) const override;
  virtual float GetCoefficientFriction(const chrono::ChVector<>& loc) const override;
};
#endif

UCLASS(Blueprintable, meta=(BlueprintSpawnableComponent) )
class CARLA_API UChronoMovementComponent : public UBaseCarlaMovementComponent
{
  GENERATED_BODY()

#ifdef WITH_CHRONO
  chrono::ChSystemNSC Sys;
  std::shared_ptr<chrono::vehicle::WheeledVehicle> Vehicle;
  TArray<std::shared_ptr<chrono::vehicle::WheeledVehicle>> Vehicles;
  TArray<ACarlaWheeledVehicle*> CarlaVehicles;
  std::shared_ptr<UERayCastTerrain> Terrain;
  TArray<std::shared_ptr<UERayCastTerrain>> Terrains;
#endif

  uint64_t MaxSubsteps = 10;
  float MaxSubstepDeltaTime = 0.01;
  FVehicleControl VehicleControl;
  FString VehicleJSON =    "sedan/vehicle/Sedan_Vehicle.json";
  FString PowertrainJSON = "sedan/powertrain/Sedan_SimpleMapPowertrain.json";
  FString TireJSON =       "sedan/tire/Sedan_TMeasyTire.json";
  FString BaseJSONPath =   "C:/sixwheel/carla/Build/chrono-install/data/vehicle/";

public:
  static void CreateChronoMovementComponentMulti(
        const TArray<ACarlaWheeledVehicle*>& Vehicles,
        uint64_t MaxSubsteps = 10,
        float MaxSubstepDeltaTime = 0.01,
        FString VehicleJSON = TEXT("sedan/vehicle/Sedan_Vehicle.json"),
        FString PowertrainJSON = TEXT("sedan/powertrain/Sedan_SimpleMapPowertrain.json"),
        FString TireJSON = TEXT("sedan/tire/Sedan_TMeasyTire.json"),
        FString BaseJSONPath = TEXT("C:/sixwheel/carla/Build/chrono-install/data/vehicle/"));

  static void CreateChronoMovementComponent(
      ACarlaWheeledVehicle* Vehicle,
      uint64_t MaxSubsteps,
      float MaxSubstepDeltaTime,
      FString VehicleJSON = "",
      FString PowertrainJSON = "",
      FString TireJSON = "",
      FString BaseJSONPath = "");

  #ifdef WITH_CHRONO
  virtual void BeginPlay() override;

  void InitializeChronoVehicle();
  void InitializeChronoVehicles();
  void ProcessControl(FVehicleControl &Control) override;

  void TickComponent(float DeltaTime,
      ELevelTick TickType,
      FActorComponentTickFunction* ThisTickFunction) override;

  void AdvanceChronoSimulation(float StepSize);

  virtual FVector GetVelocity() const override;

  virtual int32 GetVehicleCurrentGear() const override;

  virtual float GetVehicleForwardSpeed() const override;

  virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

  // Override base class methods to handle multiple vehicles
  void EnableUE4VehiclePhysics(bool bResetVelocity);
  void DisableUE4VehiclePhysics();

  // Overloaded methods to accept a vehicle parameter
  void EnableUE4VehiclePhysics(bool bResetVelocity, ACarlaWheeledVehicle* Vehicle);
  void DisableUE4VehiclePhysics(ACarlaWheeledVehicle* Vehicle);
  #endif

private:

  void DisableChronoPhysics();
  UPROPERTY()
  ACarlaWheeledVehicle* LeadVehicle = nullptr;

  UFUNCTION()
  void OnVehicleHit(AActor *Actor,
      AActor *OtherActor,
      FVector NormalImpulse,
      const FHitResult &Hit);

  // On car mesh overlap, only works when carsim is enabled
  // (this event triggers when overlapping with static environment)
  UFUNCTION()
  void OnVehicleOverlap(UPrimitiveComponent* OverlappedComponent,
      AActor* OtherActor,
      UPrimitiveComponent* OtherComp,
      int32 OtherBodyIndex,
      bool bFromSweep,
      const FHitResult & SweepResult);
  // New version that takes cached arrays
  void AdvanceChronoSimulationWithCache(
      float StepSize,
      const TArray<std::shared_ptr<chrono::vehicle::WheeledVehicle>>& CurrentVehicles,
      const TArray<std::shared_ptr<UERayCastTerrain>>& CurrentTerrains);
};
