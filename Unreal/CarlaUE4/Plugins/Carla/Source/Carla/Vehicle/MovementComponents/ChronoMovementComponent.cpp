// Copyright (c) 2021 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
// Copyright (c) 2019 Intel Corporation
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "ChronoMovementComponent.h"
#include "Carla/Vehicle/CarlaWheeledVehicle.h"
#include "Carla/Vehicle/MovementComponents/DefaultMovementComponent.h"

#include "compiler/disable-ue4-macros.h"
#include <carla/rpc/String.h>
#ifdef WITH_CHRONO
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_models/vehicle/kraz/Kraz_tractor_EngineSimpleMap.h"
#include "chrono_models/vehicle/kraz/Kraz_tractor_AutomaticTransmissionSimpleMap.h"
#include "chrono_models/vehicle/kraz/Kraz_tractor_Tire.h"
#endif
#include "compiler/enable-ue4-macros.h"
#include "Carla/Util/RayTracer.h"


void UChronoMovementComponent::CreateChronoMovementComponent(
    ACarlaWheeledVehicle* Vehicle,
    uint64_t MaxSubsteps,
    float MaxSubstepDeltaTime,
    FString VehicleJSON,
    FString PowertrainJSON,
    FString TireJSON,
    FString BaseJSONPath)
{
  #ifdef WITH_CHRONO
  UChronoMovementComponent* ChronoMovementComponent = NewObject<UChronoMovementComponent>(Vehicle);
  if (!VehicleJSON.IsEmpty())
  {
    ChronoMovementComponent->VehicleJSON = VehicleJSON;
  }
  if (!PowertrainJSON.IsEmpty())
  {
    ChronoMovementComponent->PowertrainJSON = PowertrainJSON;
  }
  if (!TireJSON.IsEmpty())
  {
    ChronoMovementComponent->TireJSON = TireJSON;
  }
  if (!BaseJSONPath.IsEmpty())
  {
    ChronoMovementComponent->BaseJSONPath = BaseJSONPath;
  }
  ChronoMovementComponent->MaxSubsteps = MaxSubsteps;
  ChronoMovementComponent->MaxSubstepDeltaTime = MaxSubstepDeltaTime;
  Vehicle->SetCarlaMovementComponent(ChronoMovementComponent);
  ChronoMovementComponent->RegisterComponent();
  #else
  UE_LOG(LogCarla, Warning, TEXT("Error: Chrono is not enabled") );
  #endif
}

#ifdef WITH_CHRONO

using namespace chrono;
using namespace chrono::vehicle;

constexpr double CMTOM = 0.01;
ChVector3d UE4LocationToChrono(const FVector& Location)
{
  return CMTOM*ChVector3d(Location.X, -Location.Y, Location.Z);
}
constexpr double MTOCM = 100;
FVector ChronoToUE4Location(const ChVector3d& position)
{
  return MTOCM*FVector(position.x(), -position.y(), position.z());
}
ChVector3d UE4DirectionToChrono(const FVector& Location)
{
  return ChVector3d(Location.X, -Location.Y, Location.Z);
}
FVector ChronoToUE4Direction(const ChVector3d& position)
{
  return FVector(position.x(), -position.y(), position.z());
}
ChQuaternion<> UE4QuatToChrono(const FQuat& Quat)
{
  return ChQuaternion<>(Quat.W, -Quat.X, Quat.Y, -Quat.Z);
}
FQuat ChronoToUE4Quat(const ChQuaternion<>& quat)
{
  return FQuat(-quat.e1(), quat.e2(), -quat.e3(), quat.e0());
}

UERayCastTerrain::UERayCastTerrain(
    ACarlaWheeledVehicle* UEVehicle,
    chrono::vehicle::ChVehicle* ChrVehicle)
    : CarlaVehicle(UEVehicle), ChronoVehicle(ChrVehicle) {}

std::pair<bool, FHitResult>
    UERayCastTerrain::GetTerrainProperties(const FVector &Location) const
{
  const double MaxDistance = 1000000;
  FVector StartLocation = Location;
  FVector EndLocation = Location + FVector(0,0,-1)*MaxDistance; // search downwards
  FHitResult Hit;
  FCollisionQueryParams CollisionQueryParams;
  CollisionQueryParams.AddIgnoredActor(CarlaVehicle);
  bool bDidHit = CarlaVehicle->GetWorld()->LineTraceSingleByChannel(
      Hit,
      StartLocation,
      EndLocation,
      ECC_GameTraceChannel2, // camera (any collision)
      CollisionQueryParams,
      FCollisionResponseParams()
  );
  return std::make_pair(bDidHit, Hit);
}

double UERayCastTerrain::GetHeight(const ChVector3d& loc) const
{
  FVector Location = ChronoToUE4Location(loc + ChVector3d(0,0,0.5)); // small offset to detect the ground properly
  auto point_pair = GetTerrainProperties(Location);
  if (point_pair.first)
  {
    double Height = CMTOM*static_cast<double>(point_pair.second.Location.Z);
    return Height;
  }
  return -1000000.0;
}
ChVector3d UERayCastTerrain::GetNormal(const ChVector3d& loc) const
{
  FVector Location = ChronoToUE4Location(loc);
  auto point_pair = GetTerrainProperties(Location);
  if (point_pair.first)
  {
    FVector Normal = point_pair.second.Normal;
    auto ChronoNormal = UE4DirectionToChrono(Normal);
    return ChronoNormal;
  }
  return UE4DirectionToChrono(FVector(0,0,1));
}
float UERayCastTerrain::GetCoefficientFriction(const ChVector3d& loc) const
{
  return 1;
}

void UChronoMovementComponent::BeginPlay()
{
  Super::BeginPlay();

  DisableUE4VehiclePhysics();

  // // // Chrono System
  Sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
  
  Sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
  Sys.GetSolver()->AsIterative()->SetMaxIterations(150);
  Sys.SetMaxPenetrationRecoverySpeed(4.0);

  InitializeChronoVehicle();

  // Create the terrain
  Terrain = chrono_types::make_shared<UERayCastTerrain>(CarlaVehicle, Vehicle.get());

  CarlaVehicle->OnActorHit.AddDynamic(
      this, &UChronoMovementComponent::OnVehicleHit);
  CarlaVehicle->GetMesh()->OnComponentBeginOverlap.AddDynamic(
      this, &UChronoMovementComponent::OnVehicleOverlap);
  CarlaVehicle->GetMesh()->SetCollisionResponseToChannel(
      ECollisionChannel::ECC_WorldStatic, ECollisionResponse::ECR_Overlap);
}

void UChronoMovementComponent::InitializeChronoVehicle()
{
  // Initial location with small offset to prevent falling through the ground
  FVector VehicleLocation = CarlaVehicle->GetActorLocation() + FVector(0,0,25);
  FQuat VehicleRotation = CarlaVehicle->GetActorRotation().Quaternion();
  auto ChronoLocation = UE4LocationToChrono(VehicleLocation);
  auto ChronoRotation = UE4QuatToChrono(VehicleRotation);


  UE_LOG(LogCarla, Log, TEXT("Loading Chrono Vehicle"));
  // Create JSON vehicle
  Vehicle = chrono_types::make_shared<kraz::Kraz_tractor>(
      &Sys,
      true);
  Vehicle->Initialize(ChCoordsys<>(ChronoLocation, ChronoRotation));
  Vehicle->GetChassis()->SetFixed(false);
  auto engine = chrono_types::make_shared<kraz::Kraz_tractor_EngineSimpleMap>("Engine");
  auto transmission = chrono_types::make_shared<kraz::Kraz_tractor_AutomaticTransmissionSimpleMap>("Transmission");
  auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
  Vehicle->InitializePowertrain(powertrain);

  // Create the tractor tires
  auto tire_FL = chrono_types::make_shared<kraz::Kraz_tractor_Tire>("TractorTire_FL");
  auto tire_FR = chrono_types::make_shared<kraz::Kraz_tractor_Tire>("TractorTire_FR");

  auto tire_RL1i = chrono_types::make_shared<kraz::Kraz_tractor_Tire>("TractorTire_RL1i");
  auto tire_RR1i = chrono_types::make_shared<kraz::Kraz_tractor_Tire>("TractorTire_RR1i");
  auto tire_RL1o = chrono_types::make_shared<kraz::Kraz_tractor_Tire>("TractorTire_RL1o");
  auto tire_RR1o = chrono_types::make_shared<kraz::Kraz_tractor_Tire>("TractorTire_RR1o");

  auto tire_RL2i = chrono_types::make_shared<kraz::Kraz_tractor_Tire>("TractorTire_RL2i");
  auto tire_RR2i = chrono_types::make_shared<kraz::Kraz_tractor_Tire>("TractorTire_RR2i");
  auto tire_RL2o = chrono_types::make_shared<kraz::Kraz_tractor_Tire>("TractorTire_RL2o");
  auto tire_RR2o = chrono_types::make_shared<kraz::Kraz_tractor_Tire>("TractorTire_RR2o");

  Vehicle->InitializeTire(tire_FL, Vehicle->GetAxle(0)->m_wheels[0], VisualizationType::NONE);
  Vehicle->InitializeTire(tire_FR, Vehicle->GetAxle(0)->m_wheels[1], VisualizationType::NONE);

  Vehicle->InitializeTire(tire_RL1i, Vehicle->GetAxle(1)->m_wheels[0], VisualizationType::NONE);
  Vehicle->InitializeTire(tire_RR1i, Vehicle->GetAxle(1)->m_wheels[1], VisualizationType::NONE);
  Vehicle->InitializeTire(tire_RL1o, Vehicle->GetAxle(1)->m_wheels[2], VisualizationType::NONE);
  Vehicle->InitializeTire(tire_RR1o, Vehicle->GetAxle(1)->m_wheels[3], VisualizationType::NONE);

  Vehicle->InitializeTire(tire_RL2i, Vehicle->GetAxle(2)->m_wheels[0], VisualizationType::NONE);
  Vehicle->InitializeTire(tire_RR2i, Vehicle->GetAxle(2)->m_wheels[1], VisualizationType::NONE);
  Vehicle->InitializeTire(tire_RL2o, Vehicle->GetAxle(2)->m_wheels[2], VisualizationType::NONE);
  Vehicle->InitializeTire(tire_RR2o, Vehicle->GetAxle(2)->m_wheels[3], VisualizationType::NONE);
  
  UE_LOG(LogCarla, Log, TEXT("Chrono vehicle initialized"));
}

void UChronoMovementComponent::ProcessControl(FVehicleControl &Control)
{
  VehicleControl = Control;
  auto PowerTrain = Vehicle->GetPowertrainAssembly();
  if (PowerTrain)
  {
    auto Transmission = PowerTrain->GetTransmission();
    auto AutoTransmission = std::dynamic_pointer_cast<ChAutomaticTransmission>(Transmission);
    if (AutoTransmission) {
      if (VehicleControl.bReverse)
      {
        AutoTransmission->SetDriveMode(ChAutomaticTransmission::DriveMode::REVERSE);
      }
      else
      {
        AutoTransmission->SetDriveMode(ChAutomaticTransmission::DriveMode::FORWARD);
      }
    }
  }
}

void UChronoMovementComponent::TickComponent(float DeltaTime,
      ELevelTick TickType,
      FActorComponentTickFunction* ThisTickFunction)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(UChronoMovementComponent::TickComponent);
  if (DeltaTime > MaxSubstepDeltaTime)
  {
    uint64_t NumberSubSteps =
        FGenericPlatformMath::FloorToInt(DeltaTime/MaxSubstepDeltaTime);
    if (NumberSubSteps < MaxSubsteps)
    {
      for (uint64_t i = 0; i < NumberSubSteps; ++i)
      {
        AdvanceChronoSimulation(MaxSubstepDeltaTime);
      }
      float RemainingTime = DeltaTime - NumberSubSteps*MaxSubstepDeltaTime;
      if (RemainingTime > 0)
      {
        AdvanceChronoSimulation(RemainingTime);
      }
    }
    else
    {
      double SubDelta = DeltaTime / MaxSubsteps;
      for (uint64_t i = 0; i < MaxSubsteps; ++i)
      {
        AdvanceChronoSimulation(SubDelta);
      }
    }
  }
  else
  {
    AdvanceChronoSimulation(DeltaTime);
  }

  const auto ChronoPositionOffset = ChVector3d(0,0,-0.25f);
  auto VehiclePos = Vehicle->GetPos() + ChronoPositionOffset;
  auto VehicleRot = Vehicle->GetRot();
  double Time = Vehicle->GetSystem()->GetChTime();

  FVector NewLocation = ChronoToUE4Location(VehiclePos);
  FQuat NewRotation = ChronoToUE4Quat(VehicleRot);
  if(NewLocation.ContainsNaN() || NewRotation.ContainsNaN())
  {
    UE_LOG(LogCarla, Warning, TEXT(
        "Error: Chrono vehicle position or rotation contains NaN. Disabling chrono physics..."));
    UDefaultMovementComponent::CreateDefaultMovementComponent(CarlaVehicle);
    return;
  }
  CarlaVehicle->SetActorLocation(NewLocation);
  FRotator NewRotator = NewRotation.Rotator();
  // adding small rotation to compensate chrono offset
  const float ChronoPitchOffset = 2.5f;
  NewRotator.Add(ChronoPitchOffset, 0.f, 0.f); 
  CarlaVehicle->SetActorRotation(NewRotator);
}

void UChronoMovementComponent::AdvanceChronoSimulation(float StepSize)
{
  double Time = Vehicle->GetSystem()->GetChTime();
  double Throttle = VehicleControl.Throttle;
  double Steering = -VehicleControl.Steer; // RHF to LHF
  double Brake = VehicleControl.Brake + VehicleControl.bHandBrake;
  Vehicle->Synchronize(Time, {Steering, Throttle, Brake}, *Terrain.get());
  Vehicle->Advance(StepSize);
  Sys.DoStepDynamics(StepSize);
}

FVector UChronoMovementComponent::GetVelocity() const
{
  if (Vehicle)
  {
    return ChronoToUE4Location(
        Vehicle->GetPointVelocity(ChVector3d(0,0,0)));
  }
  return FVector();
}

int32 UChronoMovementComponent::GetVehicleCurrentGear() const
{
  if (Vehicle)
  {
    auto PowerTrain = Vehicle->GetPowertrainAssembly();
    if (PowerTrain)
    {
      auto Transmission = PowerTrain->GetTransmission();
      if (Transmission) {
        return Transmission->GetCurrentGear();
      }
    }
  }
  return 0;
}

float UChronoMovementComponent::GetVehicleForwardSpeed() const
{
  if (Vehicle)
  {
    return GetVelocity().X;
  }
  return 0.f;
}

void UChronoMovementComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
  if(!CarlaVehicle)
  {
    return;
  }
  // reset callbacks to react to collisions
  CarlaVehicle->OnActorHit.RemoveDynamic(
      this, &UChronoMovementComponent::OnVehicleHit);
  CarlaVehicle->GetMesh()->OnComponentBeginOverlap.RemoveDynamic(
      this, &UChronoMovementComponent::OnVehicleOverlap);
  CarlaVehicle->GetMesh()->SetCollisionResponseToChannel(
      ECollisionChannel::ECC_WorldStatic, ECollisionResponse::ECR_Block);
}
#endif

void UChronoMovementComponent::DisableChronoPhysics()
{
  this->SetComponentTickEnabled(false);
  EnableUE4VehiclePhysics(true);
  CarlaVehicle->OnActorHit.RemoveDynamic(this, &UChronoMovementComponent::OnVehicleHit);
  CarlaVehicle->GetMesh()->OnComponentBeginOverlap.RemoveDynamic(
      this, &UChronoMovementComponent::OnVehicleOverlap);
  CarlaVehicle->GetMesh()->SetCollisionResponseToChannel(
      ECollisionChannel::ECC_WorldStatic, ECollisionResponse::ECR_Block);
  UDefaultMovementComponent::CreateDefaultMovementComponent(CarlaVehicle);
  carla::log_warning("Chrono physics does not support collisions yet, reverting to default PhysX physics.");
}

void UChronoMovementComponent::OnVehicleHit(AActor *Actor,
    AActor *OtherActor,
    FVector NormalImpulse,
    const FHitResult &Hit)
{
  DisableChronoPhysics();
}

// On car mesh overlap, only works when carsim is enabled
// (this event triggers when overlapping with static environment)
void UChronoMovementComponent::OnVehicleOverlap(
    UPrimitiveComponent* OverlappedComponent,
    AActor* OtherActor,
    UPrimitiveComponent* OtherComp,
    int32 OtherBodyIndex,
    bool bFromSweep,
    const FHitResult & SweepResult)
{
  if (OtherComp->GetCollisionResponseToChannel(
      ECollisionChannel::ECC_WorldDynamic) ==
      ECollisionResponse::ECR_Block)
  {
    DisableChronoPhysics();
  }
}
