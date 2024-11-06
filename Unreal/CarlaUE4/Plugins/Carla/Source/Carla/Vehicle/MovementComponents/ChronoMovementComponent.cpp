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
#include "chrono_models/vehicle/kraz/RevoyKraz.h"
#endif
#include "compiler/enable-ue4-macros.h"
#include "Carla/Util/RayTracer.h"


void UChronoMovementComponent::CreateChronoMovementComponentMulti(
    const TArray<ACarlaWheeledVehicle*>& Vehicles,
    uint64_t MaxSubsteps,
    float MaxSubstepDeltaTime,
    FString VehicleJSON,
    FString PowertrainJSON,
    FString TireJSON,
    FString BaseJSONPath)
{
  UE_LOG(LogCarla, Log, TEXT("CreateChronoMovementComponentMulti"));
  if (Vehicles.Num() != 3)
  {
      UE_LOG(LogCarla, Error, TEXT("CreateChronoMovementComponentMulti: Wrong num vehicles (need 3)"));
      return;
  }

  UE_LOG(LogCarla, Log, TEXT("CreateChronoMovementComponentMulti: ACarlaWheeledVehicle for Tractor, Revoy, Trailer"));
  // Create the ChronoMovementComponents
  ACarlaWheeledVehicle* Tractor = Vehicles[0];
  ACarlaWheeledVehicle* Revoy = Vehicles[1];
  ACarlaWheeledVehicle* Trailer = Vehicles[2];
  
  if (!Tractor || !Revoy || !Trailer)
  {
      UE_LOG(LogCarla, Error, TEXT("CreateChronoMovementComponentMulti: Some CarlaVehicle Null"));
      return;
  }

  UE_LOG(LogCarla, Log, TEXT("CreateChronoMovementComponentMulti: ChronoMovementComponenet for Tractor, Revoy, Trailer"));
  UChronoMovementComponent* TractorChronoMovementComponent = NewObject<UChronoMovementComponent>(Tractor);
  UChronoMovementComponent* RevoyChronoMovementComponent = NewObject<UChronoMovementComponent>(Revoy);
  UChronoMovementComponent* TrailerChronoMovementComponent = NewObject<UChronoMovementComponent>(Trailer);

  if (!TractorChronoMovementComponent || !RevoyChronoMovementComponent || !TrailerChronoMovementComponent)
  {
      UE_LOG(LogCarla, Error, TEXT("CreateChronoMovementComponentMulti: Some ChronoMovementComponent Null"));
      return;
  }

  // Set the properties
  UE_LOG(LogCarla, Log, TEXT("CreateChronoMovementComponentMulti: Set Tractor Properties"));
  TractorChronoMovementComponent->MaxSubsteps = MaxSubsteps;
  TractorChronoMovementComponent->RevoyType = RevoyTypeTractor;
  TractorChronoMovementComponent->MaxSubstepDeltaTime = MaxSubstepDeltaTime;
  Tractor->SetCarlaMovementComponent(TractorChronoMovementComponent);
  TractorChronoMovementComponent->RegisterComponent();


  UE_LOG(LogCarla, Log, TEXT("CreateChronoMovementComponentMulti: Set Revoy Properties"));
  RevoyChronoMovementComponent->MaxSubsteps = MaxSubsteps;
  RevoyChronoMovementComponent->RevoyType = RevoyTypeRevoy;
  RevoyChronoMovementComponent->MaxSubstepDeltaTime = MaxSubstepDeltaTime;
  Revoy->SetCarlaMovementComponent(RevoyChronoMovementComponent);
  RevoyChronoMovementComponent->RegisterComponent();


  UE_LOG(LogCarla, Log, TEXT("CreateChronoMovementComponentMulti: Set Trailer Properties"));
  TrailerChronoMovementComponent->MaxSubsteps = MaxSubsteps;
  TrailerChronoMovementComponent->RevoyType = RevoyTypeTrailer;
  TrailerChronoMovementComponent->MaxSubstepDeltaTime = MaxSubstepDeltaTime;
  Trailer->SetCarlaMovementComponent(TrailerChronoMovementComponent);
  TrailerChronoMovementComponent->RegisterComponent();  
}

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

UERayCastTerrain::UERayCastTerrain(ACarlaWheeledVehicle* UEVehicle) : CarlaVehicle(UEVehicle) {}

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
  Terrain = chrono_types::make_shared<UERayCastTerrain>(CarlaVehicle);

  CarlaVehicle->OnActorHit.AddDynamic(
      this, &UChronoMovementComponent::OnVehicleHit);
  CarlaVehicle->GetMesh()->OnComponentBeginOverlap.AddDynamic(
      this, &UChronoMovementComponent::OnVehicleOverlap);
  CarlaVehicle->GetMesh()->SetCollisionResponseToChannel(
      ECollisionChannel::ECC_WorldStatic, ECollisionResponse::ECR_Overlap);
}

// forgive me
static std::shared_ptr<kraz::RevoyKraz> RevoyKraz = nullptr;

void UChronoMovementComponent::InitializeChronoVehicle()
{

  UE_LOG(LogCarla, Log, TEXT("InitializeChronoVehicle"));
  
  FVector Location = CarlaVehicle->GetActorLocation() + FVector(0,0,25);
  FQuat Rotation = CarlaVehicle->GetActorRotation().Quaternion();
  auto ChronoLocation = UE4LocationToChrono(Location);
  auto ChronoRotation = UE4QuatToChrono(Rotation);
  ChCoordsys<> Choord(ChronoLocation, ChronoRotation);
 
  if (RevoyType != RevoyTypeNone && !RevoyKraz) {
    UE_LOG(LogCarla, Log, TEXT("First time Revoy, init static RevoyKraz"));
    RevoyKraz = chrono_types::make_shared<chrono::vehicle::kraz::RevoyKraz>(&Sys, Choord);
    RevoyKraz->Initialize();
  }
  
  // Initial location with small offset to prevent falling through the ground
  if (RevoyType == RevoyTypeTractor) {
    UE_LOG(LogCarla, Log, TEXT("Loading Kraz Tractor Vehicle"));
    Vehicle = RevoyKraz->GetTractor();
  } else if (RevoyType == RevoyTypeRevoy) {
    UE_LOG(LogCarla, Log, TEXT("Loading Kraz Revoy Vehicle"));
    Vehicle = RevoyKraz->GetRevoy();
  } else if (RevoyType == RevoyTypeTrailer) {
    UE_LOG(LogCarla, Log, TEXT("Loading Kraz Trailer Vehicle"));
    Trailer = RevoyKraz->GetTrailer();
  }
  
  UE_LOG(LogCarla, Log, TEXT("Chrono vehicle initialized"));
  
}

void UChronoMovementComponent::ProcessControl(FVehicleControl &Control)
{
  VehicleControl = Control;
  if (Vehicle)
  {
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
  auto Pos = ChVector3d(0,0,0);
  auto Rot = ChQuaternion<double>(0,0,0,1);
  double Time = 0;
  if (Vehicle) {
    Pos = Vehicle->GetPos() + ChronoPositionOffset;
    Rot = Vehicle->GetRot();
    Time = Vehicle->GetChTime();
  } else if (Trailer) {
    Pos = Trailer->GetPos() + ChronoPositionOffset;
    Rot = Trailer->GetRot();
    Time = Trailer->GetChTime();
  }

  if (RevoyType == RevoyTypeTractor) {
    UE_LOG(LogCarla, Log, TEXT("[chronomove]: Tractor %f %f %f"), Pos.x(), Pos.y(), Pos.z());
  } else if (RevoyType == RevoyTypeRevoy) {
    UE_LOG(LogCarla, Log, TEXT("[chronomove]: Revoy %f %f %f"), Pos.x(), Pos.y(), Pos.z());
  } else if (RevoyType == RevoyTypeTrailer) {
    UE_LOG(LogCarla, Log, TEXT("[chronomove]: Trailer %f %f %f"), Pos.x(), Pos.y(), Pos.z())
  }
  
  FVector NewLocation = ChronoToUE4Location(Pos);
  FQuat NewRotation = ChronoToUE4Quat(Rot);
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
  double Throttle = VehicleControl.Throttle;
  double Steering = -VehicleControl.Steer; // RHF to LHF
  double Brake = VehicleControl.Brake + VehicleControl.bHandBrake;
  if (Vehicle) {
    double Time = Vehicle->GetChTime();
    Vehicle->Synchronize(Time, {Steering, Throttle, Brake}, *Terrain.get());
    Vehicle->Advance(StepSize);
  } else if (Trailer) {
    double Time = Trailer->GetChTime();
    Trailer->Synchronize(Time, {Steering, Throttle, Brake}, *Terrain.get());
    Trailer->Advance(StepSize);
  }
  Sys.DoStepDynamics(StepSize);
}

FVector UChronoMovementComponent::GetVelocity() const
{
  if (Vehicle)
  {
    return ChronoToUE4Location(
        Vehicle->GetPointVelocity(ChVector3d(0,0,0)));
  }
  if (Trailer)
  {
    return ChronoToUE4Location(
        Trailer->GetPointVelocity(ChVector3d(0,0,0)));
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
  if (Vehicle || Trailer)
  {
    return GetVelocity().X;
  }
  return 0.f;
}

void UChronoMovementComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{

  /// free up the static chrono assembly
  RevoyKraz.reset();  
  
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
