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
    if (Vehicles.Num() == 0)
    {
        return;
    }

    // Create the ChronoMovementComponent
    ACarlaWheeledVehicle* FirstVehicle = Vehicles[0];
    if (!FirstVehicle)
    {
        return;
    }

    UChronoMovementComponent* ChronoMovement = NewObject<UChronoMovementComponent>(FirstVehicle);
    if (!ChronoMovement)
    {
        return;
    }

    // Set the properties
    ChronoMovement->MaxSubsteps = MaxSubsteps;
    ChronoMovement->MaxSubstepDeltaTime = MaxSubstepDeltaTime;
    ChronoMovement->VehicleJSON = VehicleJSON;
    ChronoMovement->PowertrainJSON = PowertrainJSON;
    ChronoMovement->TireJSON = TireJSON;
    ChronoMovement->BaseJSONPath = BaseJSONPath;

    // Store all vehicles
    ChronoMovement->CarlaVehicles = Vehicles;

    // Register and initialize the component
    ChronoMovement->RegisterComponent();
    
    // Initialize Chrono physics for all vehicles
    ChronoMovement->InitializeChronoVehicles();
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
  UE_LOG(LogCarla, Log, TEXT("CreateChronoMovementComponent: Creating ChronoMovementComponent for vehicle"));
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
ChVector<> UE4LocationToChrono(const FVector& Location)
{
  return CMTOM*ChVector<>(Location.X, -Location.Y, Location.Z);
}
constexpr double MTOCM = 100;
FVector ChronoToUE4Location(const ChVector<>& position)
{
  return MTOCM*FVector(position.x(), -position.y(), position.z());
}
ChVector<> UE4DirectionToChrono(const FVector& Location)
{
  return ChVector<>(Location.X, -Location.Y, Location.Z);
}
FVector ChronoToUE4Direction(const ChVector<>& position)
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

double UERayCastTerrain::GetHeight(const ChVector<>& loc) const
{
  FVector Location = ChronoToUE4Location(loc + ChVector<>(0,0,0.5)); // small offset to detect the ground properly
  auto point_pair = GetTerrainProperties(Location);
  if (point_pair.first)
  {
    double Height = CMTOM*static_cast<double>(point_pair.second.Location.Z);
    return Height;
  }
  return -1000000.0;
}
ChVector<> UERayCastTerrain::GetNormal(const ChVector<>& loc) const
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
float UERayCastTerrain::GetCoefficientFriction(const ChVector<>& loc) const
{
  return 1;
}

void UChronoMovementComponent::BeginPlay()
{
  Super::BeginPlay();

   // Disable UE4 physics for all vehicles
    for (ACarlaWheeledVehicle* Vehicle : CarlaVehicles)
    {
        if (Vehicle)
        {
            DisableUE4VehiclePhysics(Vehicle);
        }
    }

  
  DisableUE4VehiclePhysics();

  // // // Chrono System
  Sys.Set_G_acc(ChVector<>(0, 0, -9.81));
  Sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
  Sys.SetSolverMaxIterations(150);
  Sys.SetMaxPenetrationRecoverySpeed(4.0);

    InitializeChronoVehicles();

    // Create the terrains
    for (int32 i = 0; i < CarlaVehicles.Num(); ++i)
    {
        ACarlaWheeledVehicle* CarlaVehicle = CarlaVehicles[i];
        if (!CarlaVehicle)
        {
            continue;
        }

        auto Terrain = chrono_types::make_shared<UERayCastTerrain>(CarlaVehicle, Vehicles[i].get());
        Terrains.Add(Terrain);

        // Set up collision callbacks for each vehicle
        CarlaVehicle->OnActorHit.AddDynamic(
            this, &UChronoMovementComponent::OnVehicleHit);
        CarlaVehicle->GetMesh()->OnComponentBeginOverlap.AddDynamic(
            this, &UChronoMovementComponent::OnVehicleOverlap);
        CarlaVehicle->GetMesh()->SetCollisionResponseToChannel(
            ECollisionChannel::ECC_WorldStatic, ECollisionResponse::ECR_Overlap);
    }
}


void UChronoMovementComponent::InitializeChronoVehicles()
{
    // Set base path for vehicle JSON files
    std::string BasePath_string = carla::rpc::FromFString(BaseJSONPath);
    vehicle::SetDataPath(BasePath_string);

    // Create full paths for json files
    std::string VehicleJSON_string = carla::rpc::FromFString(VehicleJSON);
    std::string VehiclePath_string = BasePath_string + VehicleJSON_string;
    
    std::string PowertrainJSON_string = carla::rpc::FromFString(PowertrainJSON);
    std::string Powertrain_string = BasePath_string + PowertrainJSON_string;
    
    std::string TireJSON_string = carla::rpc::FromFString(TireJSON);
    std::string Tire_string = BasePath_string + TireJSON_string;

    UE_LOG(LogCarla, Log, TEXT("Loading Chrono files from: %s"), *BaseJSONPath);
    UE_LOG(LogCarla, Log, TEXT("Vehicle: %s"), *VehicleJSON);
    UE_LOG(LogCarla, Log, TEXT("Powertrain: %s"), *PowertrainJSON);
    UE_LOG(LogCarla, Log, TEXT("Tire: %s"), *TireJSON);

    // Initialize each vehicle
    for (ACarlaWheeledVehicle* CurrentVehicle : CarlaVehicles)
    {
        if (!CurrentVehicle)
        {
            Vehicles.Add(nullptr);
            Terrains.Add(nullptr);
            continue;
        }
        // Initial location with small offset to prevent falling through the ground
        FVector VehicleLocation = CurrentVehicle->GetActorLocation() + FVector(0,0,25);
        FQuat VehicleRotation = CurrentVehicle->GetActorRotation().Quaternion();
        auto ChronoLocation = UE4LocationToChrono(VehicleLocation);
        auto ChronoRotation = UE4QuatToChrono(VehicleRotation);

        // Create JSON vehicle
        auto VehiclePtr = chrono_types::make_shared<WheeledVehicle>(
            &Sys,
            VehiclePath_string);
        VehiclePtr->Initialize(ChCoordsys<>(ChronoLocation, ChronoRotation));
        VehiclePtr->GetChassis()->SetFixed(false);

        // Create and initialize the powertrain system
        auto powertrain = ReadPowertrainJSON(
            Powertrain_string);
        VehiclePtr->InitializePowertrain(powertrain);

        // Create and initialize the tires
        for (auto& axle : VehiclePtr->GetAxles()) {
            for (auto& wheel : axle->GetWheels()) {
                auto tire = ReadTireJSON(Tire_string);
                VehiclePtr->InitializeTire(tire, wheel, VisualizationType::MESH);
            }
        }
        Vehicles.Add(VehiclePtr);

        // Create terrain for this vehicle
        auto Terrain = chrono_types::make_shared<UERayCastTerrain>(CurrentVehicle, VehiclePtr.get());
        Terrains.Add(Terrain);
    }

    UE_LOG(LogCarla, Log, TEXT("Chrono vehicles initialized"));
}

void UChronoMovementComponent::InitializeChronoVehicle()
{
  // Initial location with small offset to prevent falling through the ground
  FVector VehicleLocation = CarlaVehicle->GetActorLocation() + FVector(0,0,25);
  FQuat VehicleRotation = CarlaVehicle->GetActorRotation().Quaternion();
  auto ChronoLocation = UE4LocationToChrono(VehicleLocation);
  auto ChronoRotation = UE4QuatToChrono(VehicleRotation);

  // Set base path for vehicle JSON files
  vehicle::SetDataPath("");

  std::string BasePath_string = "";

  // Create full path for json files
  // Do NOT use vehicle::GetDataFile() as strings from chrono lib
  // messes with unreal's std lib
  std::string VehicleJSON_string = "";
  std::string VehiclePath_string = "";
  FString VehicleJSONPath = "";

  std::string PowerTrainJSON_string = "";
  std::string PowerTrain_string = "";
  FString PowerTrainJSONPath = "";

  std::string TireJSON_string = "";
  std::string Tire_string = "";
  FString TireJSONPath = "";

  UE_LOG(LogCarla, Log, TEXT("Loading Chrono files: Vehicle: %s, PowerTrain: %s, Tire: %s"),
      *VehicleJSONPath,
      *PowerTrainJSONPath,
      *TireJSONPath);
  // Create JSON vehicle
  Vehicle = chrono_types::make_shared<WheeledVehicle>(
      &Sys,
      VehiclePath_string);
  Vehicle->Initialize(ChCoordsys<>(ChronoLocation, ChronoRotation));
  Vehicle->GetChassis()->SetFixed(false);
  // Create and initialize the powertrain System
  auto powertrain = ReadPowertrainJSON(
      PowerTrain_string);
  Vehicle->InitializePowertrain(powertrain);
  // Create and initialize the tires
  for (auto& axle : Vehicle->GetAxles()) {
      for (auto& wheel : axle->GetWheels()) {
          auto tire = ReadTireJSON(Tire_string);
          Vehicle->InitializeTire(tire, wheel, VisualizationType::MESH);
      }
  }
  UE_LOG(LogCarla, Log, TEXT("Chrono vehicle initialized"));
}

void UChronoMovementComponent::ProcessControl(FVehicleControl &Control)
{
  VehicleControl = Control;
  auto PowerTrain = Vehicle->GetPowertrain();
  if (PowerTrain)
  {
    if (VehicleControl.bReverse)
    {
      PowerTrain->SetDriveMode(ChPowertrain::DriveMode::REVERSE);
    }
    else
    {
      PowerTrain->SetDriveMode(ChPowertrain::DriveMode::FORWARD);
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

    const auto ChronoPositionOffset = ChVector<>(0,0,-0.25f);

    for (int32 i = 0; i < Vehicles.Num(); ++i)
    {
        auto CurrentVehicle = Vehicles[i];
        ACarlaWheeledVehicle* CarlaVehicle = CarlaVehicles[i];

        if (!CurrentVehicle || !CarlaVehicle)
        {
            continue;
        }

        auto VehiclePos = CurrentVehicle->GetVehiclePos() + ChronoPositionOffset;
        auto VehicleRot = CurrentVehicle->GetVehicleRot();

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
}

void UChronoMovementComponent::AdvanceChronoSimulation(float StepSize)
{
    double Time = Sys.GetChTime();

    // Apply controls and synchronize each vehicle
    for (int32 i = 0; i < Vehicles.Num(); ++i)
    {
        auto CurrentVehicle = Vehicles[i];
        auto CurrentTerrain = Terrains[i];

        double Throttle = VehicleControl.Throttle;
        double Steering = -VehicleControl.Steer; // RHF to LHF
        double Brake = VehicleControl.Brake + VehicleControl.bHandBrake;

        CurrentVehicle->Synchronize(Time, {Steering, Throttle, Brake}, *CurrentTerrain.get());
    }

    Sys.DoStepDynamics(StepSize);

    // Advance vehicle dynamics
    for (auto CurrentVehicle : Vehicles)
    {
        CurrentVehicle->Advance(StepSize);
    }
}
FVector UChronoMovementComponent::GetVelocity() const
{
    // Return the velocity of the first vehicle in the list
    if (Vehicles.Num() > 0 && Vehicles[0])
    {
        return ChronoToUE4Location(
            Vehicles[0]->GetVehiclePointVelocity(ChVector<>(0,0,0)));
    }
    return FVector();
}



int32 UChronoMovementComponent::GetVehicleCurrentGear() const
{
    // Return the gear of the first vehicle in the list
    if (Vehicles.Num() > 0 && Vehicles[0])
    {
        auto PowerTrain = Vehicles[0]->GetPowertrain();
        if (PowerTrain)
        {
            return PowerTrain->GetCurrentTransmissionGear();
        }
    }
    return 0;
}

float UChronoMovementComponent::GetVehicleForwardSpeed() const
{
    if (Vehicles.Num() > 0 && Vehicles[0])
    {
        return GetVelocity().X;
    }
    return 0.f;
}

void UChronoMovementComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    for (ACarlaWheeledVehicle* CarlaVehicle : CarlaVehicles)
    {
        if (!CarlaVehicle)
        {
            continue;
        }
        // Reset callbacks to react to collisions
        CarlaVehicle->OnActorHit.RemoveDynamic(
            this, &UChronoMovementComponent::OnVehicleHit);
        CarlaVehicle->GetMesh()->OnComponentBeginOverlap.RemoveDynamic(
            this, &UChronoMovementComponent::OnVehicleOverlap);
        CarlaVehicle->GetMesh()->SetCollisionResponseToChannel(
            ECollisionChannel::ECC_WorldStatic, ECollisionResponse::ECR_Block);
    }
}
#endif

void UChronoMovementComponent::DisableChronoPhysics()
{
    this->SetComponentTickEnabled(false);

    for (ACarlaWheeledVehicle* Vehicle : CarlaVehicles)
    {
        if (Vehicle)
        {
            DisableUE4VehiclePhysics(Vehicle);
            Vehicle->OnActorHit.RemoveDynamic(
                this, &UChronoMovementComponent::OnVehicleHit);
            Vehicle->GetMesh()->OnComponentBeginOverlap.RemoveDynamic(
                this, &UChronoMovementComponent::OnVehicleOverlap);
            Vehicle->GetMesh()->SetCollisionResponseToChannel(
                ECollisionChannel::ECC_WorldStatic, ECollisionResponse::ECR_Block);
            UDefaultMovementComponent::CreateDefaultMovementComponent(Vehicle);
        }
    }
    carla::log_warning("Chrono physics does not support collisions yet, reverting to default PhysX physics.");
}

void UChronoMovementComponent::EnableUE4VehiclePhysics(bool bResetVelocity)
{
    // This function shouldn't be used in this context since we're dealing with multiple vehicles.
    // You can leave it empty or log a warning.
    UE_LOG(LogCarla, Warning, TEXT("EnableUE4VehiclePhysics(bool) should not be called directly in UChronoMovementComponent."));
}

void UChronoMovementComponent::DisableUE4VehiclePhysics()
{
    // Similar to EnableUE4VehiclePhysics, leave it empty or log a warning.
    UE_LOG(LogCarla, Warning, TEXT("DisableUE4VehiclePhysics() should not be called directly in UChronoMovementComponent."));
}

void UChronoMovementComponent::EnableUE4VehiclePhysics(bool bResetVelocity, ACarlaWheeledVehicle* Vehicle)
{
    if (!Vehicle)
    {
        UE_LOG(LogCarla, Warning, TEXT("Error: Vehicle is not properly set for UChronoMovementComponent"));
        return;
    }

    FVector CurrentVelocity(0, 0, 0);
    if (!bResetVelocity)
    {
        CurrentVelocity = Vehicle->GetVelocity();
    }
    Vehicle->GetMesh()->SetPhysicsLinearVelocity(CurrentVelocity, false, "Vehicle_Base");
    Vehicle->GetVehicleMovementComponent()->SetComponentTickEnabled(true);
    Vehicle->GetVehicleMovementComponent()->Activate();
    Vehicle->GetMesh()->PhysicsTransformUpdateMode = EPhysicsTransformUpdateMode::SimulationUpatesComponentTransform;
    auto* Bone = Vehicle->GetMesh()->GetBodyInstance(NAME_None);
    if (Bone)
    {
        Bone->SetInstanceSimulatePhysics(true);
    }
    else
    {
        carla::log_warning("No bone with name");
    }
    Vehicle->GetMesh()->SetCollisionResponseToChannel(ECollisionChannel::ECC_WorldStatic, ECollisionResponse::ECR_Block);
    Vehicle->GetMesh()->SetCollisionProfileName("Vehicle");
    Vehicle->RestoreVehiclePhysicsControl();
}

void UChronoMovementComponent::DisableUE4VehiclePhysics(ACarlaWheeledVehicle* Vehicle)
{
    if (!Vehicle)
    {
        UE_LOG(LogCarla, Warning, TEXT("Error: Vehicle is not properly set for UChronoMovementComponent"));
        return;
    }

    Vehicle->GetVehicleMovementComponent()->SetComponentTickEnabled(false);
    Vehicle->GetVehicleMovementComponent()->Deactivate();
    Vehicle->GetMesh()->PhysicsTransformUpdateMode = EPhysicsTransformUpdateMode::ComponentTransformIsKinematic;
    auto* Bone = Vehicle->GetMesh()->GetBodyInstance(NAME_None);
    if (Bone)
    {
        Bone->SetInstanceSimulatePhysics(false);
    }
}

void UChronoMovementComponent::OnVehicleHit(AActor *Actor,
    AActor *OtherActor,
    FVector NormalImpulse,
    const FHitResult &Hit)
{
    DisableChronoPhysics();
}

// On car mesh overlap, only works when Chrono is enabled
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
