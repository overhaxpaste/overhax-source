#pragma once

namespace Offsets
{
	uintptr_t fnGetBounds = 0;
	uintptr_t fnLineOfSightTo = 0;
	uintptr_t fnGetBoneMatrix = 0;
	uintptr_t fnGetWeaponStats = 0;

	namespace World {
		DWORD PersistentLevel;
		DWORD Levels;
		DWORD OwningGameInstance;
	}
	namespace Level {
		DWORD Actors = 0x98;
		DWORD ActorsCount = 0xA0;
	}
	namespace GameInstance {
		DWORD LocalPlayers = 0;
	}
	namespace LocalPlayer {
		DWORD PlayerController = 0;
	}
	namespace PlayerController {
		DWORD AcknowledgedPawn = 0;
		DWORD PlayerCameraManager = 0;
	}
	namespace BuildingContainer {
		DWORD bAlreadySearched = 0;
	}
	namespace Pawn {
		DWORD RootComponent = 0;
		DWORD Mesh = 0;
		DWORD bIsDying = 0;
		DWORD RemoteViewPitch = 0;
		DWORD Weapon = 0;
		DWORD PlayerState = 0;
	}
	namespace SceneComponent {
		DWORD RelativeLocation = 0;
		DWORD ComponentVelocity = 0;
	}
	namespace PlayerState {
		DWORD TeamIndex = 0;
	}
	namespace Weapon {
		DWORD LastFireAbilityTime = 0;
	}
}