// MultiChronoMovementComponent.cpp

#include "MultiChronoMovementComponent.h"
#include "Carla/Vehicle/CarlaWheeledVehicle.h"

#include "compiler/disable-ue4-macros.h"
#include <carla/rpc/String.h>
#ifdef WITH_CHRONO
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#endif
#include "compiler/enable-ue4-macros.h"

#include "Carla/Util/RayTracer.h"

UMultiChronoMovementComponent::UMultiChronoMovementComponent()
{
#ifdef WITH_CHRONO
  PrimaryComponentTick.bCanEverTick = true;
#endif
}

void UMultiChronoMovementComponent::CreateMultiChronoMovementComponent(
    const TArray<ACarlaWheeledVehicle*>& Vehicles,
    uint64_t MaxSubsteps,
    float MaxSubstepDeltaTime,
    const FString& VehicleJSON,
    const FString& PowertrainJSON,
    const FString& TireJSON,
    const FString& BaseJSONPath)
{
#ifdef WITH_CHRONO
  if (Vehicles.Num() == 0)
  {
    UE_LOG(LogCarla, Warning, TEXT("No vehicles provided for MultiChronoMovementComponent."));
    return;
  }

  UMultiChronoMovementComponent* ChronoMovementComponent =
      NewObject<UMultiChronoMovementComponent>(Vehicles[0]);

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
  ChronoMovementComponent->CarlaVehicles = Vehicles;

  // Set this component as the movement component for each vehicle
  for (ACarlaWheeledVehicle* Vehicle : Vehicles)
  {
    Vehicle->SetCarlaMovementComponent(ChronoMovementComponent);
  }

  ChronoMovementComponent->RegisterComponent();
#else
  UE_LOG(LogCarla, Warning, TEXT("Error: Chrono is not enabled"));
#endif
}

#ifdef WITH_CHRONO

using namespace chrono;
using namespace chrono::vehicle;

UMultiChronoMovementComponent::UMultiChronoMovementComponent()
{
  PrimaryComponentTick.bCanEverTick = true;
}

void UMultiChronoMovementComponent::BeginPlay()
{
  Super::BeginPlay();

  // Disable UE4 vehicle physics for all vehicles
  for (ACarlaWheeledVehicle* Vehicle : CarlaVehicles)
  {
    Vehicle->GetVehicleMovementComponent()->SetComponentTickEnabled(false);
  }

  // Initialize the Chrono system
  Sys.Set_G_acc(ChVector<>(0, 0, -9.81));
  Sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
  Sys.SetSolverMaxIterations(150);
  Sys.SetMaxPenetrationRecoverySpeed(4.0);

  InitializeChronoVehicles();

  // Create the terrain
  Terrain = chrono_types::make_shared<UERayCastTerrainMulti>();

  // Set up collision callbacks
  for (ACarlaWheeledVehicle* Vehicle : CarlaVehicles)
  {
    Vehicle->OnActorHit.AddDynamic(this, &UMultiChronoMovementComponent::OnVehicleHit);
    Vehicle->GetMesh()->OnComponentBeginOverlap.AddDynamic(this, &UMultiChronoMovementComponent::OnVehicleOverlap);
    Vehicle->GetMesh()->SetCollisionResponseToChannel(
        ECollisionChannel::ECC_WorldStatic, ECollisionResponse::ECR_Overlap);
  }
}

void UMultiChronoMovementComponent::InitializeChronoVehicles()
{
  // Set base path for vehicle JSON files
  vehicle::SetDataPath(carla::rpc::FromFString(BaseJSONPath));
  std::string BasePathString = carla::rpc::FromFString(BaseJSONPath);

  // Create full paths for JSON files
  std::string VehicleJSONString = carla::rpc::FromFString(VehicleJSON);
  std::string VehiclePathString = BasePathString + VehicleJSONString;

  std::string PowertrainJSONString = carla::rpc::FromFString(PowertrainJSON);
  std::string PowertrainPathString = BasePathString + PowertrainJSONString;

  std::string TireJSONString = carla::rpc::FromFString(TireJSON);
  std::string TirePathString = BasePathString + TireJSONString;

  FString VehicleJSONPath = carla::rpc::ToFString(VehiclePathString);
  FString PowertrainJSONPath = carla::rpc::ToFString(PowertrainPathString);
  FString TireJSONPath = carla::rpc::ToFString(TirePathString);

  UE_LOG(LogCarla, Log, TEXT("Loading Chrono files: Vehicle: %s, Powertrain: %s, Tire: %s"),
      *VehicleJSONPath, *PowertrainJSONPath, *TireJSONPath);

  // Initialize Chrono vehicles for each Carla vehicle
  for (ACarlaWheeledVehicle* CarlaVehicle : CarlaVehicles)
  {
    // Initial location with small offset to prevent falling through the ground
    FVector VehicleLocation = CarlaVehicle->GetActorLocation() + FVector(0, 0, 25);
    FQuat VehicleRotation = CarlaVehicle->GetActorRotation().Quaternion();
    auto ChronoLocation = ChVector<>(VehicleLocation.X * 0.01, -VehicleLocation.Y * 0.01, VehicleLocation.Z * 0.01);
    auto ChronoRotation = ChQuaternion<>(VehicleRotation.W, -VehicleRotation.X, VehicleRotation.Y, -VehicleRotation.Z);

    // Create JSON vehicle
    auto Vehicle = chrono_types::make_shared<WheeledVehicle>(&Sys, VehiclePathString);
    Vehicle->Initialize(ChCoordsys<>(ChronoLocation, ChronoRotation));
    Vehicle->GetChassis()->SetFixed(false);

    // Create and initialize the powertrain system
    auto Powertrain = ReadPowertrainJSON(PowertrainPathString);
    Vehicle->InitializePowertrain(Powertrain);

    // Create and initialize the tires
    for (auto& Axle : Vehicle->GetAxles())
    {
      for (auto& Wheel : Axle->GetWheels())
      {
        auto Tire = ReadTireJSON(TirePathString);
        Vehicle->InitializeTire(Tire, Wheel, VisualizationType::MESH);
      }
    }

    ChronoVehicles.Add(CarlaVehicle, Vehicle);
  }

  UE_LOG(LogCarla, Log, TEXT("Chrono vehicles initialized"));
}

void UMultiChronoMovementComponent::ProcessControl(FVehicleControl& Control)
{
  VehicleControl = Control;

  // Process control inputs for each vehicle
  for (auto& Elem : ChronoVehicles)
  {
    auto Powertrain = Elem.Value->GetPowertrain();
    if (Powertrain)
    {
      if (VehicleControl.bReverse)
      {
        Powertrain->SetDriveMode(ChPowertrain::DriveMode::REVERSE);
      }
      else
      {
        Powertrain->SetDriveMode(ChPowertrain::DriveMode::FORWARD);
      }
    }
  }
}

void UMultiChronoMovementComponent::TickComponent(
    float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(UMultiChronoMovementComponent::TickComponent);

  if (DeltaTime > MaxSubstepDeltaTime)
  {
    uint64_t NumberSubSteps = static_cast<uint64_t>(FMath::FloorToInt(DeltaTime / MaxSubstepDeltaTime));
    NumberSubSteps = FMath::Min(NumberSubSteps, MaxSubsteps);

    for (uint64_t i = 0; i < NumberSubSteps; ++i)
    {
      AdvanceChronoSimulation(MaxSubstepDeltaTime);
    }
    float RemainingTime = DeltaTime - NumberSubSteps * MaxSubstepDeltaTime;
    if (RemainingTime > 0)
    {
      AdvanceChronoSimulation(RemainingTime);
    }
  }
  else
  {
    AdvanceChronoSimulation(DeltaTime);
  }

  // Update positions and rotations for all vehicles
  for (ACarlaWheeledVehicle* CarlaVehicle : CarlaVehicles)
  {
    auto ChronoVehicle = ChronoVehicles[CarlaVehicle];
    auto VehiclePos = ChronoVehicle->GetVehiclePos();
    auto VehicleRot = ChronoVehicle->GetVehicleRot();

    FVector NewLocation = FVector(VehiclePos.x() * 100.0, -VehiclePos.y() * 100.0, VehiclePos.z() * 100.0);
    FQuat NewRotation = FQuat(-VehicleRot.e1(), VehicleRot.e2(), -VehicleRot.e3(), VehicleRot.e0());

    if (NewLocation.ContainsNaN() || NewRotation.ContainsNaN())
    {
      UE_LOG(LogCarla, Warning, TEXT(
          "Error: Chrono vehicle position or rotation contains NaN. Disabling Chrono physics..."));
      DisableChronoPhysics();
      return;
    }

    CarlaVehicle->SetActorLocation(NewLocation);
    FRotator NewRotator = NewRotation.Rotator();
    CarlaVehicle->SetActorRotation(NewRotator);
  }
}

void UMultiChronoMovementComponent::AdvanceChronoSimulation(float StepSize)
{
  // STUB: Implement the simulation advancement for all vehicles

  // Example stub implementation:
  // for (auto& Elem : ChronoVehicles)
  // {
  //   auto Vehicle = Elem.Value;
  //   // Get control inputs
  //   // Create driver inputs
  //   // Synchronize and advance the vehicle
  // }
  // Advance the Chrono system
  Sys.DoStepDynamics(StepSize);
}

FVector UMultiChronoMovementComponent::GetVelocity() const
{
  // STUB: Return the velocity of one of the vehicles or an average velocity
  return FVector::ZeroVector;
}

int32 UMultiChronoMovementComponent::GetVehicleCurrentGear() const
{
  // STUB: Return the gear of one of the vehicles
  return 0;
}

float UMultiChronoMovementComponent::GetVehicleForwardSpeed() const
{
  // STUB: Return the forward speed of one of the vehicles
  return 0.0f;
}

void UMultiChronoMovementComponent::DisableChronoPhysics()
{
  // Disable the component tick
  SetComponentTickEnabled(false);

  // Enable UE4 vehicle physics for all vehicles
  for (ACarlaWheeledVehicle* Vehicle : CarlaVehicles)
  {
    Vehicle->GetVehicleMovementComponent()->SetComponentTickEnabled(true);
  }

  // Remove collision callbacks
  for (ACarlaWheeledVehicle* Vehicle : CarlaVehicles)
  {
    Vehicle->OnActorHit.RemoveDynamic(this, &UMultiChronoMovementComponent::OnVehicleHit);
    Vehicle->GetMesh()->OnComponentBeginOverlap.RemoveDynamic(this, &UMultiChronoMovementComponent::OnVehicleOverlap);
    Vehicle->GetMesh()->SetCollisionResponseToChannel(
        ECollisionChannel::ECC_WorldStatic, ECollisionResponse::ECR_Block);
  }

  UE_LOG(LogCarla, Warning, TEXT("Disabled Chrono physics, reverting to default physics."));
}

void UMultiChronoMovementComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
  // Reset collision callbacks
  for (ACarlaWheeledVehicle* Vehicle : CarlaVehicles)
  {
    Vehicle->OnActorHit.RemoveDynamic(this, &UMultiChronoMovementComponent::OnVehicleHit);
    Vehicle->GetMesh()->OnComponentBeginOverlap.RemoveDynamic(this, &UMultiChronoMovementComponent::OnVehicleOverlap);
    Vehicle->GetMesh()->SetCollisionResponseToChannel(
        ECollisionChannel::ECC_WorldStatic, ECollisionResponse::ECR_Block);
  }

  Super::EndPlay(EndPlayReason);
}

void UMultiChronoMovementComponent::OnVehicleHit(
    AActor* Actor, AActor* OtherActor, FVector NormalImpulse, const FHitResult& Hit)
{
  // DisableChronoPhysics();
}

void UMultiChronoMovementComponent::OnVehicleOverlap(
    UPrimitiveComponent* OverlappedComponent,
    AActor* OtherActor,
    UPrimitiveComponent* OtherComp,
    int32 OtherBodyIndex,
    bool bFromSweep,
    const FHitResult& SweepResult)
{
  // Figure out what to do here
}

UERayCastTerrainMulti::UERayCastTerrainMulti()
{
  // STUB: Initialize the terrain if needed
}

double UERayCastTerrainMulti::GetHeight(const ChVector<>& loc) const
{
  // STUB: Return the terrain height at the given location
  return 0.0;
}

ChVector<> UERayCastTerrainMulti::GetNormal(const ChVector<>& loc) const
{
  // STUB: Return the terrain normal at the given location
  return ChVector<>(0, 0, 1);
}

float UERayCastTerrainMulti::GetCoefficientFriction(const ChVector<>& loc) const
{
  // STUB: Return the friction coefficient at the given location
  return 1.0f;
}

#endif