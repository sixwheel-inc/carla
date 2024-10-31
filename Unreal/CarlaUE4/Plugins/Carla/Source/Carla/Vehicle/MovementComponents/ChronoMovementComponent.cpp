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
    // Set the first vehicle as the lead vehicle
    ChronoMovement->LeadVehicle = FirstVehicle;
    // Set the properties
    ChronoMovement->MaxSubsteps = MaxSubsteps;
    ChronoMovement->MaxSubstepDeltaTime = MaxSubstepDeltaTime;
    ChronoMovement->VehicleJSON = VehicleJSON;
    ChronoMovement->PowertrainJSON = PowertrainJSON;
    ChronoMovement->TireJSON = TireJSON;
    ChronoMovement->BaseJSONPath = BaseJSONPath;

    // Store all vehicles
    ChronoMovement->CarlaVehicles = Vehicles;
    for (ACarlaWheeledVehicle* Vehicle : Vehicles)
    {
        Vehicle->SetCarlaMovementComponent(ChronoMovement);
    }

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
    if (Vehicles.Num() > 0) {
      UE_LOG(LogCarla, Warning, TEXT("Vehicles already initialized. Skipping initialization."));
      return;
    }
    // Clear existing vehicles first to prevent duplicates
    Vehicles.Empty();
    Terrains.Empty();

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

    UE_LOG(LogCarla, Log, TEXT("Initializing Chrono vehicles. Carla vehicles count: %d"), CarlaVehicles.Num());

    // Initialize each vehicle
    for (ACarlaWheeledVehicle* CurrentVehicle : CarlaVehicles)
    {
        if (!CurrentVehicle)
        {
            UE_LOG(LogCarla, Warning, TEXT("Skipping null Carla vehicle"));
            continue;  // Skip instead of adding null entries
        }

        try 
        {
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
            auto powertrain = ReadPowertrainJSON(Powertrain_string);
            VehiclePtr->InitializePowertrain(powertrain);

            // Create and initialize the tires
            for (auto& axle : VehiclePtr->GetAxles()) {
                for (auto& wheel : axle->GetWheels()) {
                    auto tire = ReadTireJSON(Tire_string);
                    VehiclePtr->InitializeTire(tire, wheel, VisualizationType::MESH);
                }
            }
            Vehicles.Add(VehiclePtr);
                        // Create and initialize terrain for this vehicle
            auto TerrainPtr = chrono_types::make_shared<UERayCastTerrain>(CurrentVehicle, VehiclePtr.get());
            Terrains.Add(TerrainPtr);

        }
        catch (const std::exception& e)
        {
            UE_LOG(LogCarla, Error, TEXT("Failed to initialize Chrono vehicle: %s"), 
                UTF8_TO_TCHAR(e.what()));
            continue;  // Skip this vehicle instead of adding null entries
        }
    }

    // Ensure arrays match
    if (Vehicles.Num() != CarlaVehicles.Num())
    {
        UE_LOG(LogCarla, Error, TEXT("Vehicle initialization mismatch. Chrono: %d, Carla: %d"), 
            Vehicles.Num(), CarlaVehicles.Num());
        // Clear everything if initialization failed
        Vehicles.Empty();
        Terrains.Empty();
        DisableChronoPhysics();
        return;
    }

    UE_LOG(LogCarla, Log, TEXT("Chrono vehicles initialized successfully. Count: %d"), Vehicles.Num());
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
  // VehicleControl = Control;
  // auto PowerTrain = Vehicle->GetPowertrain();
  // if (PowerTrain)
  // {
  //   if (VehicleControl.bReverse)
  //   {
  //     PowerTrain->SetDriveMode(ChPowertrain::DriveMode::REVERSE);
  //   }
  //   else
  //   {
  //     PowerTrain->SetDriveMode(ChPowertrain::DriveMode::FORWARD);
  //   }
  // }
}

void UChronoMovementComponent::TickComponent(float DeltaTime,
      ELevelTick TickType,
      FActorComponentTickFunction* ThisTickFunction)
{
    TRACE_CPUPROFILER_EVENT_SCOPE(UChronoMovementComponent::TickComponent);
    // Check if this instance is handling the lead vehicle. If not, perform no-op.
    if (GetOwner() != LeadVehicle)
    {
        UE_LOG(LogCarla, Log, TEXT("Not the lead vehicle, skipping tick"));
        return;
    }

    // Cache the vehicle arrays at the start of tick
    TArray<std::shared_ptr<WheeledVehicle>> CurrentVehicles = Vehicles;
    TArray<std::shared_ptr<UERayCastTerrain>> CurrentTerrains = Terrains;

    UE_LOG(LogCarla, Log, TEXT("TickComponent - Vehicles: %d, CarlaVehicles: %d"), 
        CurrentVehicles.Num(), CarlaVehicles.Num());

    // Early exit if no vehicles
    if (CurrentVehicles.Num() == 0 || CarlaVehicles.Num() == 0)
    {
        UE_LOG(LogCarla, Warning, TEXT("No vehicles available for tick"));
        return;
    }

    // Process physics timestep with cached arrays
    if (DeltaTime > MaxSubstepDeltaTime)
    {
        uint64_t NumberSubSteps =
            FGenericPlatformMath::FloorToInt(DeltaTime/MaxSubstepDeltaTime);
        if (NumberSubSteps < MaxSubsteps)
        {
            for (uint64_t i = 0; i < NumberSubSteps; ++i)
            {
                AdvanceChronoSimulationWithCache(MaxSubstepDeltaTime, CurrentVehicles, CurrentTerrains);
            }
            float RemainingTime = DeltaTime - NumberSubSteps*MaxSubstepDeltaTime;
            if (RemainingTime > 0)
            {
                AdvanceChronoSimulationWithCache(RemainingTime, CurrentVehicles, CurrentTerrains);
            }
        }
        else
        {
            double SubDelta = DeltaTime / MaxSubsteps;
            for (uint64_t i = 0; i < MaxSubsteps; ++i)
            {
                AdvanceChronoSimulationWithCache(SubDelta, CurrentVehicles, CurrentTerrains);
            }
        }
    }
    else
    {
        AdvanceChronoSimulationWithCache(DeltaTime, CurrentVehicles, CurrentTerrains);
    }
        // Update vehicle positions in UE4 after all physics steps are complete
    for (int32 i = 0; i < CurrentVehicles.Num(); ++i)
    {
        if (!CurrentVehicles[i] || !CarlaVehicles.IsValidIndex(i) || !CarlaVehicles[i])
        {
            continue;
        }

        const auto ChronoPositionOffset = ChVector<>(0,0,-0.25f);
        auto VehiclePos = CurrentVehicles[i]->GetVehiclePos() + ChronoPositionOffset;
        auto VehicleRot = CurrentVehicles[i]->GetVehicleRot();
        double Time = CurrentVehicles[i]->GetSystem()->GetChTime();
            // Logging Vehicle Position and Rotation
        UE_LOG(LogCarla, Log, TEXT("VehiclePos: X=%f, Y=%f, Z=%f"), VehiclePos.x(), VehiclePos.y(), VehiclePos.z());
        UE_LOG(LogCarla, Log, TEXT("VehicleRot: W=%f, X=%f, Y=%f, Z=%f"), VehicleRot.e0(), VehicleRot.e1(), VehicleRot.e2(), VehicleRot.e3());

        FVector NewLocation = ChronoToUE4Location(VehiclePos);
        UE_LOG(LogCarla, Log, TEXT("NewLocation: X=%f, Y=%f, Z=%f"), NewLocation.X, NewLocation.Y, NewLocation.Z);
        FQuat NewRotation = ChronoToUE4Quat(VehicleRot);
        UE_LOG(LogCarla, Log, TEXT("NewRotation: W=%f, X=%f, Y=%f, Z=%f"), NewRotation.W, NewRotation.X, NewRotation.Y, NewRotation.Z);
        if(NewLocation.ContainsNaN() || NewRotation.ContainsNaN())
        {
            UE_LOG(LogCarla, Warning, TEXT(
                "Error: Chrono vehicle %d position or rotation contains NaN. Disabling chrono physics..."), i);
            UDefaultMovementComponent::CreateDefaultMovementComponent(CarlaVehicles[i]);
            continue;
        }

        CarlaVehicles[i]->SetActorLocation(NewLocation);
        FRotator NewRotator = NewRotation.Rotator();
        // adding small rotation to compensate chrono offset
        const float ChronoPitchOffset = 2.5f;
        NewRotator.Add(ChronoPitchOffset, 0.f, 0.f);
        CarlaVehicles[i]->SetActorRotation(NewRotator);
    }
    UE_LOG(LogCarla, Log, TEXT("TickComponent - Finished updating vehicle positions"));
}

// Keep the original implementation for backward compatibility
void UChronoMovementComponent::AdvanceChronoSimulation(float StepSize)
{
    AdvanceChronoSimulationWithCache(StepSize, Vehicles, Terrains);
}

// New implementation that uses cached arrays
void UChronoMovementComponent::AdvanceChronoSimulationWithCache(
    float StepSize,
    const TArray<std::shared_ptr<chrono::vehicle::WheeledVehicle>>& CurrentVehicles,
    const TArray<std::shared_ptr<UERayCastTerrain>>& CurrentTerrains)
{
    if (CurrentVehicles.Num() == 0 || CurrentTerrains.Num() == 0)
    {
        UE_LOG(LogCarla, Warning, TEXT("AdvanceChronoSimulation - No vehicles or terrains available"));
        UE_LOG(LogCarla, Log, TEXT("CurrentVehicles: %d, CurrentTerrains: %d"), CurrentVehicles.Num(), CurrentTerrains.Num());
        return;
    }

    double Time = Sys.GetChTime();

    // Use the cached arrays instead of member variables
    for (int32 i = 0; i < CurrentVehicles.Num(); ++i)
    {
        auto CurrentVehicle = CurrentVehicles[i];
        auto CurrentTerrain = CurrentTerrains[i];
        
        if (!CurrentVehicle || !CurrentTerrain)
        {
            continue;
        }

        double Throttle = VehicleControl.Throttle;
        double Steering = -VehicleControl.Steer;
        double Brake = VehicleControl.Brake + VehicleControl.bHandBrake;

        CurrentVehicle->Synchronize(Time, {Steering, Throttle, Brake}, *CurrentTerrain.get());
    }

    Sys.DoStepDynamics(StepSize);

    for (auto Vehicle : CurrentVehicles)
    {
        if (Vehicle)
        {
            Vehicle->Advance(StepSize);
        }
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
    // Use atomic operation to prevent multiple simultaneous calls
    static std::atomic<bool> DisableInProgress(false);
    bool expected = false;
    if (!DisableInProgress.compare_exchange_strong(expected, true))
    {
        UE_LOG(LogCarla, Warning, TEXT("DisableChronoPhysics already in progress"));
        return;
    }

    this->SetComponentTickEnabled(false);

    // Take a copy of the vehicles array to prevent issues if it's modified during iteration
    TArray<ACarlaWheeledVehicle*> VehiclesToDisable = CarlaVehicles;
    
    for (ACarlaWheeledVehicle* Vehicle : VehiclesToDisable)
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

    // Clear arrays after processing
    Vehicles.Empty();
    Terrains.Empty();
    CarlaVehicles.Empty();

    carla::log_warning("Chrono physics disabled, reverted to default PhysX physics.");
    DisableInProgress = false;
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

