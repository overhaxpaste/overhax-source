#define _CRT_SECURE_NO_WARNINGS
#define COBJMACROS
#define WIN32_LEAN_AND_MEAN
#define MAX_VERTEX_BUFFER 512 * 1024
#define MAX_INDEX_BUFFER 128 * 1024
#define NK_INCLUDE_STANDARD_IO
#define NK_INCLUDE_STANDARD_VARARGS
#define NK_INCLUDE_DEFAULT_ALLOCATOR
#define NK_INCLUDE_FIXED_TYPES
#define NK_INCLUDE_VERTEX_BUFFER_OUTPUT
#define NK_INCLUDE_FONT_BAKING
#define NK_INCLUDE_DEFAULT_FONT

#define NK_D3D11_IMPLEMENTATION
#define NK_IMPLEMENTATION

#include <windows.h>
#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <time.h>
#include <math.h>
#include "utils.h"
#include <D3D11.h>
#include <d3dx11.h>
#include <d3dx10.h>
#include <codecvt>
#include <Psapi.h>
#include <list>
#include "skCrypter.h"
#include <Wininet.h>

#include "FW1FontWrapper.h"
#pragma comment(lib, "FW1FontWrapper.lib")

#include "spoof_call.h"
#include "xorstr.hpp"

#include "lazyimporter.h"
#include "memory.h"

#include "Nuklear/nuklear.h"
#include "Nuklear/demo/d3d11/nuklear_d3d11.h"
#include "Draw.hpp"

#include "Offsets.h"
#include "FortRenderer.h"
#include "FortUpdater.h"
#include "Exploits/SpeedHack.h"

#include <tchar.h>
#include <conio.h>
#include <stdio.h>
#include <stdlib.h>
#include <ostream>
#include <regex>
#include <winuser.h>
#include <WinReg.h>
#include <winternl.h>

#include <TlHelp32.h>
#include <random>
#include <ctime>
#include <urlmon.h>

#pragma comment(lib, "Psapi.lib")
#pragma comment(lib, "d3dx11.lib")
#pragma comment (lib, "d3dx10.lib")
#pragma comment(lib, "d3d11.lib")
#pragma comment(lib, "Advapi32.lib")
#pragma comment(lib, "urlmon.lib")
#pragma comment(lib, "wininet.lib")
#pragma comment(lib, "ntdll.lib")

#define skCrypt(str) _xor_(str).c_str()
#define BIT_CHECK(a,b) (!!((a) & (1ULL<<(b))))

uint64_t OFFSET_UOBJECT = NULL;
uint64_t OFFSET_UWORLD = NULL;
uint64_t OFFSET_GETOBJECTNAMES = NULL;
uint64_t OFFSET_GETNAMEBYINDEX = NULL;
uint64_t OFFSET_FNFREE = NULL;

nk_context* g_pNkContext;
static ID3D11Device* uDevice;
uint64_t entityx;

Vector3 CamLoc;
Vector3 CamRot;
float GlobalFOV = 80.f;

nk_color Boss_color = { 255,0,255,255 };
nk_color Bot_color = { 255,165,0,255 };
nk_color Enemy_color = { 255,0,0,255 };
nk_color Team_color = { 0,255,0,255 };

uintptr_t OldAimingActor = 0;

struct FBox
{
	Vector3  Min;
	Vector3  Max;
	unsigned char IsValid;
	unsigned char UnknownData00[0x3];
};

struct FMinimalViewInfo
{
	Vector3 Loc;
	Vector3 Rot;
	float FOV;
};

struct FMatrix
{
	float M[4][4];
};

BOOL IsValid(DWORD64 address)
{
	if (!spoof_call(game_rbx_jmp, IsBadReadPtr, (const void*)address, (UINT_PTR)8)) return TRUE;
	else return FALSE;
}

template<typename T>
T read(DWORD_PTR address, const T& def = T())
{
	if (IsValid(address))
		return *(T*)address;
	else
		return T();
}

uint64_t base_address;

DWORD_PTR Uworld;
DWORD_PTR LocalPawn;
DWORD_PTR LocalWeapon;
DWORD_PTR PlayerCameraManager;
DWORD_PTR Localplayer;
DWORD_PTR Rootcomp;
DWORD_PTR PawnMesh;
DWORD_PTR PlayerController;
DWORD_PTR Ulevel;
DWORD_PTR Levels;
int LevelsCount;

DWORD_PTR AActors;
int actor_count;

int Actors[2] = { 0,0 };
int Bots[2] = { 0,0 };

float X;
float Y;

static nkDraw* Draw = new nkDraw();

static auto Dummy_vTable = new uintptr_t[0x1001];

bool GetAimKey()
{
	switch (AimKey)
	{
	case Keys::LButton:
		return (o_getasynckeystate((DWORD)VK_LBUTTON) & 0x8000);
	case Keys::RButton:			  
		return (o_getasynckeystate((DWORD)VK_RBUTTON) & 0x8000);
	case Keys::Alt_Button:		  
		return (o_getasynckeystate((DWORD)VK_MENU) & 0x8000);
	case Keys::Shift:			  
		return (o_getasynckeystate((DWORD)VK_LSHIFT) & 0x8000);
	default:
		return false;
	}
}

D3DMATRIX MatrixMultiplication(D3DMATRIX pM1, D3DMATRIX pM2)
{
	D3DMATRIX pOut;
	pOut._11 = pM1._11 * pM2._11 + pM1._12 * pM2._21 + pM1._13 * pM2._31 + pM1._14 * pM2._41;
	pOut._12 = pM1._11 * pM2._12 + pM1._12 * pM2._22 + pM1._13 * pM2._32 + pM1._14 * pM2._42;
	pOut._13 = pM1._11 * pM2._13 + pM1._12 * pM2._23 + pM1._13 * pM2._33 + pM1._14 * pM2._43;
	pOut._14 = pM1._11 * pM2._14 + pM1._12 * pM2._24 + pM1._13 * pM2._34 + pM1._14 * pM2._44;
	pOut._21 = pM1._21 * pM2._11 + pM1._22 * pM2._21 + pM1._23 * pM2._31 + pM1._24 * pM2._41;
	pOut._22 = pM1._21 * pM2._12 + pM1._22 * pM2._22 + pM1._23 * pM2._32 + pM1._24 * pM2._42;
	pOut._23 = pM1._21 * pM2._13 + pM1._22 * pM2._23 + pM1._23 * pM2._33 + pM1._24 * pM2._43;
	pOut._24 = pM1._21 * pM2._14 + pM1._22 * pM2._24 + pM1._23 * pM2._34 + pM1._24 * pM2._44;
	pOut._31 = pM1._31 * pM2._11 + pM1._32 * pM2._21 + pM1._33 * pM2._31 + pM1._34 * pM2._41;
	pOut._32 = pM1._31 * pM2._12 + pM1._32 * pM2._22 + pM1._33 * pM2._32 + pM1._34 * pM2._42;
	pOut._33 = pM1._31 * pM2._13 + pM1._32 * pM2._23 + pM1._33 * pM2._33 + pM1._34 * pM2._43;
	pOut._34 = pM1._31 * pM2._14 + pM1._32 * pM2._24 + pM1._33 * pM2._34 + pM1._34 * pM2._44;
	pOut._41 = pM1._41 * pM2._11 + pM1._42 * pM2._21 + pM1._43 * pM2._31 + pM1._44 * pM2._41;
	pOut._42 = pM1._41 * pM2._12 + pM1._42 * pM2._22 + pM1._43 * pM2._32 + pM1._44 * pM2._42;
	pOut._43 = pM1._41 * pM2._13 + pM1._42 * pM2._23 + pM1._43 * pM2._33 + pM1._44 * pM2._43;
	pOut._44 = pM1._41 * pM2._14 + pM1._42 * pM2._24 + pM1._43 * pM2._34 + pM1._44 * pM2._44;

	return pOut;
}

D3DXMATRIX Matrix(Vector3 rot, Vector3 origin = Vector3(0, 0, 0))
{
	float radPitch = (rot.x * float(M_PI) / 180.f);
	float radYaw = (rot.y * float(M_PI) / 180.f);
	float radRoll = (rot.z * float(M_PI) / 180.f);

	float SP = sinf(radPitch);
	float CP = cosf(radPitch);
	float SY = sinf(radYaw);
	float CY = cosf(radYaw);
	float SR = sinf(radRoll);
	float CR = cosf(radRoll);

	D3DMATRIX matrix;
	matrix.m[0][0] = CP * CY;
	matrix.m[0][1] = CP * SY;
	matrix.m[0][2] = SP;
	matrix.m[0][3] = 0.f;

	matrix.m[1][0] = SR * SP * CY - CR * SY;
	matrix.m[1][1] = SR * SP * SY + CR * CY;
	matrix.m[1][2] = -SR * CP;
	matrix.m[1][3] = 0.f;

	matrix.m[2][0] = -(CR * SP * CY + SR * SY);
	matrix.m[2][1] = CY * SR - CR * SP * SY;
	matrix.m[2][2] = CR * CP;
	matrix.m[2][3] = 0.f;

	matrix.m[3][0] = origin.x;
	matrix.m[3][1] = origin.y;
	matrix.m[3][2] = origin.z;
	matrix.m[3][3] = 1.f;

	return matrix;
}

static FMatrix* myMatrix = new FMatrix();
Vector3 GetBoneLocByIdx(uintptr_t mesh, int id)
{
	if (!mesh) return { 0,0,0 };

	auto fGetBoneMatrix = ((FMatrix*(__fastcall*)(uintptr_t, FMatrix*, int))(Offsets::fnGetBoneMatrix));
	spoof_call(game_rbx_jmp, fGetBoneMatrix, mesh, myMatrix, id);

	return Vector3(myMatrix->M[3][0], myMatrix->M[3][1], myMatrix->M[3][2]);
}

Vector3 ProjectWorldToScreen(Vector3 WorldLocation, Vector3 camrot)
{
	Vector3 Screenlocation = Vector3(0, 0, 0);
	Vector3 Rotation = camrot;

	D3DMATRIX tempMatrix = Matrix(Rotation);

	Vector3 vAxisX, vAxisY, vAxisZ;

	vAxisX = Vector3(tempMatrix.m[0][0], tempMatrix.m[0][1], tempMatrix.m[0][2]);
	vAxisY = Vector3(tempMatrix.m[1][0], tempMatrix.m[1][1], tempMatrix.m[1][2]);
	vAxisZ = Vector3(tempMatrix.m[2][0], tempMatrix.m[2][1], tempMatrix.m[2][2]);

	Vector3 vDelta = WorldLocation - CamLoc;
	Vector3 vTransformed = Vector3(vDelta.Dot(vAxisY), vDelta.Dot(vAxisZ), vDelta.Dot(vAxisX));

	float FovAngle = GlobalFOV;
	float ScreenCenterX = X / 2.0f;
	float ScreenCenterY = Y / 2.0f;

	if (vTransformed.z < 1.f || tanf(FovAngle * (float)M_PI / 360.f) == 0.f) return Vector3(0, 0, 0);

	Screenlocation.x = ScreenCenterX + vTransformed.x * (ScreenCenterX / tanf(FovAngle * (float)M_PI / 360.f)) / vTransformed.z;
	Screenlocation.y = ScreenCenterY - vTransformed.y * (ScreenCenterX / tanf(FovAngle * (float)M_PI / 360.f)) / vTransformed.z;

	return Screenlocation;
}

Vector3 CalcAim();

typedef uintptr_t(__fastcall* Cam)(uintptr_t, FMinimalViewInfo*);
Cam o_GetCameraInfo = NULL;
uintptr_t hkGetCameraInfo(uintptr_t a1, FMinimalViewInfo* a2)
{
	static float last_fire_ability_time = 0.f;

	uintptr_t ret = spoof_call(game_rbx_jmp, o_GetCameraInfo, a1, a2);

	CamLoc = a2->Loc;
	CamRot = a2->Rot;

	if (FOVSlider)
	{
		a2->FOV = (float)FOVMoment;
	}

	GlobalFOV = a2->FOV;

	if (bSilent)
	{
		if (!IsValid(LocalPawn) || !IsValid(entityx)) return ret;

		LocalWeapon = read<uintptr_t>(LocalPawn + Offsets::Pawn::Weapon);

		if (LocalWeapon)
		{
			if (*(float*)(LocalWeapon + Offsets::Weapon::LastFireAbilityTime) != last_fire_ability_time)
			{
				Vector3 SilentAimAngle = CalcAim();

				if (SilentAimAngle.x != 0 || SilentAimAngle.y != 0)
					a2->Rot = SilentAimAngle;

				last_fire_ability_time = *(float*)(LocalWeapon + Offsets::Weapon::LastFireAbilityTime);
			}
		}
	}

	return ret;
}

bool InstallCameraHook(uintptr_t PlayerCameraManager)
{
	uintptr_t vTable_address = *(uintptr_t*)(PlayerCameraManager);

	if ((uintptr_t)hkGetCameraInfo == read<uintptr_t>(vTable_address + (0xCC * 0x8))) return true;

	if (!vTable_address) return false;

	int num_virtual_methods = 0;
	int z = 0;
	while (read<uintptr_t>(vTable_address + (z * 0x8))) {
		num_virtual_methods += 1;
		z++;
	}

	for (int i = 0; i < num_virtual_methods; i++)
	{
		Dummy_vTable[i] = read<uintptr_t>(vTable_address + (i * 0x8));
	}

	o_GetCameraInfo = (Cam)(read<uintptr_t>(vTable_address + (0xCC * 8)));

	Dummy_vTable[0xCC] = (uintptr_t)hkGetCameraInfo;

	*(uintptr_t**)(PlayerCameraManager) = Dummy_vTable;

	return true;
}

void* SetHook_1(void* pSource, void* pDestination, int dwLen)
{
	DWORD MinLen = 14;

	if (dwLen < MinLen) return NULL;

	BYTE stub[] = {
	0xFF, 0x25, 0x00, 0x00, 0x00, 0x00, // jmp qword ptr [$+6]
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 // ptr
	};

	static const void* pTrampoline = nullptr;
	if (!pTrampoline) {
		const auto ntdll = reinterpret_cast<const unsigned char*>(spoof_call(game_rbx_jmp, GetModuleHandleW, (LPCWSTR)NULL));
		const auto dos = reinterpret_cast<const IMAGE_DOS_HEADER*>(ntdll);
		const auto nt = reinterpret_cast<const IMAGE_NT_HEADERS*>(ntdll + dos->e_lfanew);
		const auto sections = IMAGE_FIRST_SECTION(nt);
		const auto num_sections = nt->FileHeader.NumberOfSections;

		constexpr char section_name[5]{ '.', 't', 'e', 'x', 't' };
		const auto     section = std::find_if(sections, sections + num_sections, [&](const auto& s) {
			return std::equal(s.Name, s.Name + 5, section_name);
			});

		constexpr unsigned char instr_bytes[2]{ 0xFF, 0x25 };
		const auto              va = ntdll + section->VirtualAddress;
		pTrampoline = std::search(va, va + section->Misc.VirtualSize, instr_bytes, instr_bytes + 2);
	}

	DWORD64 retto = (DWORD64)pSource + dwLen;

	// trampoline
	memcpy(stub + 6, &retto, 8);
	memcpy((void*)((DWORD_PTR)pTrampoline), pSource, dwLen);
	memcpy((void*)((DWORD_PTR)pTrampoline + dwLen), stub, sizeof(stub));

	// orig
	memcpy(stub + 6, &pDestination, 8);
	memcpy(pSource, stub, sizeof(stub));

	for (int i = MinLen; i < dwLen; i++)
	{
		*(BYTE*)((DWORD_PTR)pSource + i) = 0x90;
	}

	return (void*)((DWORD_PTR)pTrampoline);
}

void FreeObjName(__int64 address)
{
	if (!IsValid(address)) return;

	auto func = reinterpret_cast<__int64(__fastcall*)(__int64 a1)>(OFFSET_FNFREE);

	spoof_call(game_rbx_jmp, func, address);
}

std::string GetObjectName(uintptr_t Object) {

	if (Object == NULL)
		return skCrypt("");

	auto fGetObjName = reinterpret_cast<FString * (__fastcall*)(FString * name, uintptr_t entity)>(OFFSET_GETOBJECTNAMES);

	FString result;
	spoof_call(game_rbx_jmp, fGetObjName, &result, Object);

	if (result.c_str() == NULL)
		return skCrypt("");

	auto result_str = result.ToString();

	if (result.c_str() != NULL)
		FreeObjName((__int64)result.c_str());

	return result_str;
}

Vector3 GetPawnEyeViewRot(__int64 Entity)
{
	if (!Entity || !read<uintptr_t>(Entity)) return Vector3(0, 0, 0);

	Vector3 out, out_Rot;

	auto GetActorEyesViewPoint = (*(void(__fastcall**)(__int64, Vector3*, Vector3*))(*(uint64_t*)Entity + 0x5F8));
	spoof_call(game_rbx_jmp, GetActorEyesViewPoint, Entity, &out, &out_Rot);

	return out_Rot;
}

bool IsTargetVisible(uintptr_t entity)
{
	if (!entity || !LocalPawn || !LocalWeapon || !PawnMesh || !PlayerController) return false;

	Vector3 tmp = { 0,0,0 };

	auto fLineOfSight = ((BOOL(__fastcall*)(uintptr_t, uintptr_t, Vector3*))(Offsets::fnLineOfSightTo));
	return spoof_call(game_rbx_jmp, fLineOfSight, PlayerController, entity, &tmp);
}

FBox GetFBox(uintptr_t Actor)
{
	if (!Actor) return {};

	Vector3 Origin, BoxExtend;

	auto fGetActorBounds = reinterpret_cast<void(__fastcall*)(__int64, char, Vector3*, Vector3*)>(Offsets::fnGetBounds);

	spoof_call(game_rbx_jmp, fGetActorBounds, (__int64)Actor, (char)true, &Origin, &BoxExtend);

	FBox NewBox;
	NewBox.IsValid = 1;
	NewBox.Min = Origin - BoxExtend;
	NewBox.Max = Origin + BoxExtend;

	return NewBox;
}

Vector3 CalcAim()
{
	Vector3 RetVector = { 0,0,0 };

	if (!entityx) return { 0,0,0 };

	uint64_t currentactormesh = read<uint64_t>(entityx + Offsets::Pawn::Mesh);
	if (!currentactormesh) return { 0,0,0 };
	
	Vector3 rootHead = GetBoneLocByIdx(currentactormesh, 66);
	if (rootHead.x == 0 && rootHead.y == 0) return Vector3(0, 0, 0);

	Vector3 VectorPos = rootHead - CamLoc;
	
	float distance = VectorPos.Length();
	RetVector.x = -(((float)acos(VectorPos.z / distance) * (float)(180.0f / M_PI)) - 90.f);
	RetVector.y = (float)atan2(VectorPos.y, VectorPos.x) * (float)(180.0f / M_PI);
	
	return RetVector;
}

double GetCrossDistance(double x1, double y1, double x2, double y2)
{
	return spoof_call(game_rbx_jmp, sqrtf, spoof_call(game_rbx_jmp, powf, (float)(x1-x2), (float)2) + spoof_call(game_rbx_jmp, powf, (float)(y1-y2), (float)2));
}

bool GetClosestPlayerToCrossHair(Vector3 Pos, float& max, DWORD_PTR entity)
{
	if (entity)
	{
		float Dist = GetCrossDistance(Pos.x, Pos.y, (X / 2), (Y / 2));

		if (Dist < max)
		{

			//GlobalFOV : X = NewFOV : NewSize
			//NewSize = NewFOV * X / GlobalFOV

			float Radius = (AimFov * X / GlobalFOV) / 2;

			if (Pos.x <= ((X / 2) + Radius) &&
				Pos.x >= ((X / 2) - Radius) &&
				Pos.y <= ((Y / 2) + Radius) &&
				Pos.y >= ((Y / 2) - Radius))
			{
				max = Dist;
				entityx = entity;
				return true;
			}

			return false;
		}
	}

	return false;
}

void CheckClosestFOVEntity(DWORD_PTR entity, Vector3 Localcam, float& max)
{
	if (!entity || !LocalPawn || !LocalWeapon) return;

	uint64_t currentactormesh = read<uint64_t>(entity + Offsets::Pawn::Mesh);
	if (!currentactormesh) return;

	Vector3 rootHead = GetBoneLocByIdx(currentactormesh, 66);
	Vector3 rootHeadOut = ProjectWorldToScreen(rootHead, CamRot);

	if (rootHeadOut.x <= 0 || rootHeadOut.y <= 0) return;
	if (rootHeadOut.x >= X || rootHeadOut.y >= Y) return;

	if (bVisible && !IsTargetVisible(entity)) return;

	if (GetClosestPlayerToCrossHair(rootHeadOut, max, entity))
		entityx = entity;
}

void CheckClosestDistEntity(DWORD_PTR entity, Vector3 Localcam, float& max)
{
	if (!entity) return;

	uint64_t currentactormesh = read<uint64_t>(entity + Offsets::Pawn::Mesh);
	if (!currentactormesh) return;

	Vector3 rootHead = GetBoneLocByIdx(currentactormesh, 66);

	if (rootHead.x == 0 && rootHead.y == 0) return;

	if (bVisible && !IsTargetVisible(entity)) return;

	Vector3 Delta = rootHead - CamLoc;

	float Dist = spoof_call(game_rbx_jmp, sqrtf, spoof_call(game_rbx_jmp, powf, (Delta.x), (float)2) + spoof_call(game_rbx_jmp, powf, (Delta.y), (float)2) + spoof_call(game_rbx_jmp, powf, (Delta.z), (float)2));

	if (AimDistance != 300)
	{
		if ((Dist * 0.01f) > (float)AimDistance) return;
	}

	if ((Dist * 0.01f) < max)
	{
		max = (Dist * 0.01f);
		entityx = entity;
		return;
	}
	return;
}

void Draw3DBoundingBox(uintptr_t mesh, uint64_t CurrentActor, nk_color color)
{
	if (!CurrentActor) return;

	Vector3 min, max, vec1, vec2, vec3, vec4, vec5, vec6, vec7, vec8;

	FBox box = GetFBox(CurrentActor);

	if (!box.IsValid) return;

	min = box.Min;
	max = box.Max;

	vec3 = min;
	vec3.x = max.x;
	vec4 = min;
	vec4.y = max.y;
	vec5 = min;
	vec5.z = max.z;
	vec6 = max;
	vec6.x = min.x;
	vec7 = max;
	vec7.y = min.y;
	vec8 = max;
	vec8.z = min.z;
	vec1 = ProjectWorldToScreen(min, CamRot);
	vec2 = ProjectWorldToScreen(max, CamRot);
	vec3 = ProjectWorldToScreen(vec3, CamRot);
	vec4 = ProjectWorldToScreen(vec4, CamRot);
	vec5 = ProjectWorldToScreen(vec5, CamRot);
	vec6 = ProjectWorldToScreen(vec6, CamRot);
	vec7 = ProjectWorldToScreen(vec7, CamRot);
	vec8 = ProjectWorldToScreen(vec8, CamRot);

	if (vec1.x == 0 && vec1.y == 0) return;
	if (vec2.x == 0 && vec2.y == 0) return;
	if (vec3.x == 0 && vec3.y == 0) return;
	if (vec4.x == 0 && vec4.y == 0) return;
	if (vec5.x == 0 && vec5.y == 0) return;
	if (vec6.x == 0 && vec6.y == 0) return;
	if (vec7.x == 0 && vec7.y == 0) return;
	if (vec8.x == 0 && vec8.y == 0) return;

	Draw->DrawLine(vec1.x, vec1.y, vec5.x, vec5.y, 1, color);
	Draw->DrawLine(vec2.x, vec2.y, vec8.x, vec8.y, 1, color);
	Draw->DrawLine(vec3.x, vec3.y, vec7.x, vec7.y, 1, color);
	Draw->DrawLine(vec4.x, vec4.y, vec6.x, vec6.y, 1, color);
	Draw->DrawLine(vec1.x, vec1.y, vec3.x, vec3.y, 1, color);
	Draw->DrawLine(vec1.x, vec1.y, vec4.x, vec4.y, 1, color);
	Draw->DrawLine(vec8.x, vec8.y, vec3.x, vec3.y, 1, color);
	Draw->DrawLine(vec8.x, vec8.y, vec4.x, vec4.y, 1, color);
	Draw->DrawLine(vec2.x, vec2.y, vec6.x, vec6.y, 1, color);
	Draw->DrawLine(vec2.x, vec2.y, vec7.x, vec7.y, 1, color);
	Draw->DrawLine(vec5.x, vec5.y, vec6.x, vec6.y, 1, color);
	Draw->DrawLine(vec5.x, vec5.y, vec7.x, vec7.y, 1, color);
}

void Draw2DBoundingBox(Vector3 StartBoxLoc, float flWidth, float Height, nk_color color)
{
	StartBoxLoc.x = StartBoxLoc.x - (flWidth / 2);

	Draw->DrawLine(StartBoxLoc.x, StartBoxLoc.y, StartBoxLoc.x + flWidth, StartBoxLoc.y,1,  color); //bottom
	Draw->DrawLine(StartBoxLoc.x, StartBoxLoc.y, StartBoxLoc.x, StartBoxLoc.y + Height,1, color); //left
	Draw->DrawLine(StartBoxLoc.x + flWidth, StartBoxLoc.y, StartBoxLoc.x + flWidth, StartBoxLoc.y + Height,1, color); //right
	Draw->DrawLine(StartBoxLoc.x, StartBoxLoc.y + Height, StartBoxLoc.x + flWidth, StartBoxLoc.y + Height,1, color); //up
}

void DrawSkeleton(DWORD_PTR mesh)
{
	std::list<int> upper_part = { 65,66 };
	std::list<int> right_arm = { 65, BONE_R_ARM_TOP, BONE_R_ARM_LOWER, BONE_MISC_R_HAND_1 };
	std::list<int> left_arm = { 65, BONE_L_ARM_TOP, BONE_L_ARM_LOWER, BONE_MISC_L_HAND };
	std::list<int> spine = { 65, BONE_PELVIS_1 };
	std::list<int> lower_right = { BONE_PELVIS_2, BONE_R_THIGH ,76 };
	std::list<int> lower_left = { BONE_PELVIS_2, BONE_L_THIGH ,69 };
	std::list<std::list<int>> Skeleton = { upper_part, right_arm, left_arm, spine, lower_right, lower_left };

	if (!mesh) return;

	nk_color col = { 0,255,255,255 };

	Vector3 neckpos = GetBoneLocByIdx(mesh, 65);
	Vector3 pelvispos = GetBoneLocByIdx(mesh, BONE_PELVIS_2);

	Vector3 previous(0, 0, 0);
	Vector3 current, p1, c1;

	for (auto a : Skeleton)
	{
		previous = Vector3(0, 0, 0);
		for (int bone : a)
		{
			current = bone == 65 ? neckpos : (bone == BONE_PELVIS_2 ? pelvispos : GetBoneLocByIdx(mesh, bone));
			if (previous.x == 0.f)
			{
				previous = current;
				continue;
			}

			p1 = ProjectWorldToScreen(previous, CamRot);
			c1 = ProjectWorldToScreen(current, CamRot);


			if (p1.x != 0 && p1.y != 0 && c1.x != 0 && c1.y != 0)
				Draw->DrawLine(p1.x, p1.y, c1.x, c1.y, 1, col);

			previous = current;
		}
	}
}

void inline DrawLaser(uintptr_t target, uintptr_t currentactormesh, nk_color color)
{
	if (target)
	{
		Vector3 target_rot = GetPawnEyeViewRot(target);
		Vector3 target_loc = GetBoneLocByIdx(currentactormesh, 66);

		if (target_rot.x == 0 && target_rot.y == 0) return;
		if (target_loc.x == 0 && target_loc.y == 0) return;

		double radiantsYAW = ((target_rot.y * M_PI) / 180);  //Trasformazione da Gradi a Radianti.

		float RemotePitch = *(float*)(target + Offsets::Pawn::RemoteViewPitch);
		double radiantsPITCH = (((float)RemotePitch * M_PI) / 180);
		double hyp = 250;
		double X = (hyp * cos(radiantsYAW));
		double Y = (hyp * sin(radiantsYAW));
		double Z = (hyp * sin(radiantsPITCH));
		float PlX = target_loc.x + X;
		float PlZ = target_loc.y + Y;
		float PlY = target_loc.z + Z;

		Vector3 Laser, Laser_out, HeadLoc_out;
		Laser.x = PlX;
		Laser.z = PlY;
		Laser.y = PlZ;

		Laser_out = ProjectWorldToScreen(Laser, CamRot);
		HeadLoc_out = ProjectWorldToScreen(target_loc, CamRot);

		if (Laser_out.x == 0 && Laser_out.y == 0) return;
		if (HeadLoc_out.x == 0 && HeadLoc_out.y == 0) return;

		Draw->DrawLine(HeadLoc_out.x, HeadLoc_out.y, Laser_out.x, Laser_out.y, 1, color);
	}
}

void inline DrawChest(uintptr_t chest)
{
	if (!chest) return;

	uintptr_t RootComponent = read<uintptr_t>(chest + Offsets::Pawn::RootComponent);
	if (!RootComponent) return;

	Vector3 CenterLocation = *(Vector3*)(RootComponent + Offsets::SceneComponent::RelativeLocation);

	Vector3 Out_Screen = ProjectWorldToScreen(CenterLocation, CamRot);

	if (Out_Screen.x == 0 && Out_Screen.y == 0) return;

	Draw->nkDrawText(skCrypt("[CHEST]"), Out_Screen.x, Out_Screen.y, 7, nk_rgb(255, 255, 255));
}

void inline DrawAmmoBox(uintptr_t ammo)
{
	if (!ammo) return;

	uintptr_t RootComponent = read<uintptr_t>(ammo + Offsets::Pawn::RootComponent);
	if (!RootComponent) return;

	Vector3 CenterLocation = *(Vector3*)(RootComponent + Offsets::SceneComponent::RelativeLocation);

	Vector3 Out_Screen = ProjectWorldToScreen(CenterLocation, CamRot);

	if (Out_Screen.x == 0 && Out_Screen.y == 0) return;

	Draw->nkDrawText(skCrypt("[AMMO BOX]"), Out_Screen.x, Out_Screen.y, 10, nk_rgb(255, 255, 255));
}

void inline DrawLlama(uintptr_t llama)
{
	if (!llama) return;

	uintptr_t RootComponent = read<uintptr_t>(llama + Offsets::Pawn::RootComponent);
	if (!RootComponent) return;

	Vector3 CenterLocation = *(Vector3*)(RootComponent + Offsets::SceneComponent::RelativeLocation);

	Vector3 Out_Screen = ProjectWorldToScreen(CenterLocation, CamRot);

	if (Out_Screen.x == 0 && Out_Screen.y == 0) return;

	Draw->nkDrawText(skCrypt("[LLAMA]"), Out_Screen.x, Out_Screen.y, 7, nk_rgb(255,0,255));
}

Vector3 Normalize(Vector3 vec)
{
	float a1 = vec.x;
	float a2 = vec.y;
	float a3 = vec.z;
	double length = vec.Length();

	if (length == 0.0) return { 0,0,0 };

	return Vector3(a1 / length, a2 / length, a3 / length);
}

Vector3 inline LimitRotation(Vector3 startRotation, Vector3 endRotation)
{
	Vector3 ret;
	auto scale = AimSmooth;
	auto currentRotation = startRotation;

	ret.x = (endRotation.x - startRotation.x) / scale + startRotation.x;
	ret.y = (endRotation.y - startRotation.y) / scale + startRotation.y;

	return ret;
}

void DoAimbot(Vector3 Localcam)
{
	if (!read<uint64_t>(entityx + Offsets::Pawn::RootComponent))
	{
		entityx = 0;
		return;
	}
	if (!read<uint64_t>(entityx + Offsets::Pawn::PlayerState))
	{
		entityx = 0;
		return;
	}
	if (!read<uint64_t>(entityx + Offsets::Pawn::Mesh))
	{
		entityx = 0;
		return;
	}
	uint8_t bIsDying = *(uint8_t*)(entityx + Offsets::Pawn::bIsDying);
	if (BIT_CHECK(bIsDying, 3))
	{
		entityx = 0;
		return;
	}

	if (!PlayerController || !read<uintptr_t>(PlayerController))
	{
		entityx = 0;
		return;
	}

	Vector3 NewAngle = CalcAim();

	if (NewAngle.x == 0 && NewAngle.y == 0) return;

	if (AimSmooth > 0)
		NewAngle = LimitRotation(CamRot, NewAngle);

	NewAngle.z = 0;

	if (bAimLock)
	{
		auto SetIgnoreLookInput = (*(void(__fastcall**)(uint64_t, char bNewLookInput))(*(uint64_t*)PlayerController + 0x728));
		spoof_call(game_rbx_jmp, SetIgnoreLookInput, PlayerController, (char)1);
	}

	auto ClientSetRotation = (*(void(__fastcall**)(uint64_t, Vector3, char))(*(uint64_t*)PlayerController + 0x628));
	spoof_call(game_rbx_jmp, ClientSetRotation, PlayerController, NewAngle, (char)0);
}

void DrawAimingEnemy()
{
	if (!entityx) return;

	uint64_t currentactormesh = read<uint64_t>(entityx + Offsets::Pawn::Mesh);
	if (!currentactormesh) return;

	nk_color col = { 0,255,255,255 };

	Vector3 target_loc = GetBoneLocByIdx(currentactormesh, 66);
	if (target_loc.x == 0 && target_loc.y == 0) return;

	Vector3 HeadLoc_out = ProjectWorldToScreen(target_loc, CamRot);

	if (HeadLoc_out.x == 0 && HeadLoc_out.y == 0) return;

	Draw->DrawLine(HeadLoc_out.x, HeadLoc_out.y, X/2, Y/2, 1, col);
}

Vector3 WorldToRadar(Vector3 Location, INT RadarX, INT RadarY, int size)
{
	Vector3 Return;

	FLOAT CosYaw = spoof_call(game_rbx_jmp, cosf, (float)((CamRot.y) * M_PI / 180.f));
	FLOAT SinYaw = spoof_call(game_rbx_jmp, sinf, (float)((CamRot.y) * M_PI / 180.f));

	FLOAT DeltaX = Location.x - CamLoc.x;
	FLOAT DeltaY = Location.y - CamLoc.y;

	FLOAT LocationX = (DeltaY * CosYaw - DeltaX * SinYaw) / (200);
	FLOAT LocationY = (DeltaX * CosYaw + DeltaY * SinYaw) / (200);

	if (LocationX > ((size / 2) - 5.0f) - 2.5f)
		LocationX = ((size / 2) - 5.0f) - 2.5f;
	else if (LocationX < -(((size / 2) - 5.0f) - 2.5f))
		LocationX = -(((size / 2) - 5.0f) - 2.5f);

	if (LocationY > ((size / 2) - 5.0f) - 2.5f)
		LocationY = ((size / 2) - 5.0f) - 2.5f;
	else if (LocationY < -(((size / 2) - 5.0f) - 2.5f))
		LocationY = -(((size / 2) - 5.0f) - 2.5f);

	Return.x = LocationX + RadarX;
	Return.y = -LocationY + RadarY;

	return Return;
}

void RadarDraw(int Size)
{
	nk_color grey;
	grey.a = 127; grey.b = 0; grey.g = 0; grey.r = 0;
	nk_color greylight;
	greylight.a = 127; greylight.b = 80; greylight.g = 80; greylight.r = 80;
	//BOX
	Draw->DrawFilledRect(1200, 10, Size, Size, 0.f, grey);
	Draw->DrawFilledRect(1201, 11, Size - 2, Size - 2, 0.f, greylight);

	//CROSS
	Draw->DrawLine(1200 + (Size / 2), 10, 1200 + (Size / 2), 10 + Size, 1, grey);
	Draw->DrawLine(1200, 10 + (Size / 2), 1200 + Size, 10 + (Size / 2), 1, grey);

	if (!PlayerController
		|| !PlayerCameraManager) return;

	//DefFOV : Size = NewFOV : NewSize -> NewSize = NewFOV * Size / DefFOV
	float NewFOV = (GlobalFOV * Size) / 90.f;

	//Draw FOV
	if (NewFOV <= Size)
	{
		Draw->DrawLine(1200 + (Size / 2), 10 + (Size / 2), 1200 + (Size / 2) + (NewFOV / 2), 10, 1, grey);
		Draw->DrawLine(1200 + (Size / 2), 10 + (Size / 2), 1200 + (Size / 2) - (NewFOV / 2), 10, 1, grey);
	}
	else
	{
		//AllX : PlusSize = FullHeight : NewHeigh -> NewHeigh = FullHeight * PlusSize / AllX
		float NewHeight = (Size / 2) * (NewFOV - (Size / 2)) / NewFOV;


		Draw->DrawLine(1200 + (Size / 2), 10 + (Size / 2), 1200, 10 + NewHeight, 1, grey);
		Draw->DrawLine(1200 + (Size / 2), 10 + (Size / 2), 1200 + Size, 10 + NewHeight, 1, grey);
	}

	nk_color black;
	black.a = 255; black.b = 0; black.g = 0; black.r = 0;
	nk_color white;
	white.a = 255; white.b = 255; white.g = 255; white.r = 255;

	//My Player
	Draw->DrawFilledRect(1200 + (Size / 2) - 2, 10 + (Size / 2) - 2, 4, 4, 0.f, black);
	Draw->DrawFilledRect(1200 + (Size / 2) - 2, 10 + (Size / 2) - 2, 2, 2, 0.f, white);
}
//Orignal = 500
void AddTargetToRadar(Vector3 ActorLoc, int RadarSize, nk_color col)
{
	Vector3 RadarCoords = WorldToRadar(ActorLoc, 1200 + (RadarSize / 2), 10 + (RadarSize / 2), RadarSize - 2);

	nk_color color;
	color.a = 255; color.b = 0; color.g = 0; color.r = 0;

	Draw->DrawFilledRect(RadarCoords.x - 2, RadarCoords.y - 2, 4, 4, 0.f, color);
	Draw->DrawFilledRect(RadarCoords.x - 1, RadarCoords.y - 1, 2, 2, 0.f, col);
}

void RadarLoop()
{
	nk_color BOSScol;
	BOSScol.r = 255; BOSScol.g = 0; BOSScol.b = 255; BOSScol.a = 255;

	nk_color BOTcol;
	BOTcol.r = 255; BOTcol.g = 165; BOTcol.b = 0; BOTcol.a = 255;

	nk_color col;
	col.r = 255; col.g = 0; col.b = 0; col.a = 255;

	nk_color TEAMcol;
	TEAMcol.r = 0; TEAMcol.g = 255; TEAMcol.b = 0; TEAMcol.a = 255;

	int MyTeamID = 0;
	int TeamID = 1;

	if (!LocalPawn) return;

	uintptr_t MyPlayerState = read<uint64_t>(LocalPawn + Offsets::Pawn::PlayerState);
	if (MyPlayerState) MyTeamID = *(int*)(MyPlayerState + Offsets::PlayerState::TeamIndex);

	RadarDraw(300);

	for (int i = 0; i < actor_count; i++)
	{
		uint64_t CurrentActor = read<uint64_t>(AActors + i * 0x8);
		if (CurrentActor == (uint64_t)nullptr || CurrentActor == -1 || CurrentActor == NULL) continue;

		int curactorid = *(int*)(CurrentActor + 0x18);

		if (curactorid == 0) continue;

		//Enemy
		if (CurrentActor != LocalPawn && CurrentActor != Localplayer && CurrentActor != PlayerController)
		{
			if (!CurrentActor) continue;

			if (curactorid == Actors[0] || curactorid == Actors[1])
			{
				uint8_t bIsDying = *(uint8_t*)(CurrentActor + Offsets::Pawn::bIsDying);
				if (BIT_CHECK(bIsDying, 3)) continue;

				uintptr_t PlayerState = read<uint64_t>(CurrentActor + Offsets::Pawn::PlayerState);
				if (PlayerState) TeamID = *(int*)(PlayerState + Offsets::PlayerState::TeamIndex);

				uint64_t RootComponents = read<uint64_t>(CurrentActor + Offsets::Pawn::RootComponent);
				if (!RootComponents) continue;

				Vector3 Headpos = *(Vector3*)(RootComponents + Offsets::SceneComponent::RelativeLocation);

				if (TeamID != MyTeamID)
					AddTargetToRadar(Headpos, 300, col);
				else
					AddTargetToRadar(Headpos, 300, TEAMcol);
			}
			else
			{
				if (curactorid == Bots[0] || (curactorid >= (Bots[1] - 5) && curactorid <= (Bots[1] + 5)))
				{
					uint8_t bIsDying = *(uint8_t*)(CurrentActor + Offsets::Pawn::bIsDying);
					if (BIT_CHECK(bIsDying, 3)) continue;

					uint64_t RootComponents = read<uint64_t>(CurrentActor + Offsets::Pawn::RootComponent);
					if (!RootComponents) continue;

					Vector3 Headpos = *(Vector3*)(RootComponents + Offsets::SceneComponent::RelativeLocation);

					if (curactorid == Bots[0])
						AddTargetToRadar(Headpos, 300, BOTcol);
					else
						AddTargetToRadar(Headpos, 300, BOSScol);
				}
			}
		}
	}
}

void OnlyActorsLoop()
{
	if (!IsValid(LocalPawn) || !IsValid(LocalWeapon)) return;

	float FOVmax = 9999.f;
	float DistanceMax = 9999999999.f;

	int MyTeamID = 0;
	int TeamID = 1;

	for (int i = 0; i < actor_count; i++)
	{
		uint64_t CurrentActor = read<uint64_t>(AActors + i * 0x8);
		if (!CurrentActor) continue;

		int curactorid = *(int*)(CurrentActor + 0x18);
		if (curactorid == 0) continue;

		//Enemy
		if (CurrentActor != LocalPawn && CurrentActor != Localplayer && CurrentActor != PlayerController)
		{
			if (Actors[0] == 0 || Actors[1] == 0 || Bots[0] == 0 || Bots[1] == 0)
			{
				if (curactorid != Actors[0] &&
					curactorid != Actors[1] &&
					curactorid != Bots[0] &&
					curactorid != Bots[1])
				{
					char* ObjName = (char*)malloc(200);

					strcpy(ObjName, (char*)GetObjectName(CurrentActor).c_str());

					if (strstr((const char*)ObjName, (const char*)skCrypt("PlayerPawn_Athena_C")))
						Actors[0] = curactorid;

					if (strstr((const char*)ObjName, (const char*)skCrypt("BP_PlayerPawn_Athena_Phoebe_C")))
						Actors[1] = curactorid;

					if (strstr((const char*)ObjName, (const char*)skCrypt("BP_MangPlayerPawn_Default_C")))
						Bots[0] = curactorid;

					if (strstr((const char*)ObjName, (const char*)skCrypt("BP_MangPlayerPawn_Boss_")))
						Bots[1] = curactorid;

					free(ObjName);
				}
			}

			if ((curactorid == Actors[0] || curactorid == Actors[1]) && curactorid != 0)
			{
				uintptr_t MyPlayerState = read<uint64_t>(LocalPawn + Offsets::Pawn::PlayerState);
				if (MyPlayerState) MyTeamID = *(int*)(MyPlayerState + Offsets::PlayerState::TeamIndex);

				uint8_t bIsDying = *(uint8_t*)(CurrentActor + Offsets::Pawn::bIsDying);
				if (BIT_CHECK(bIsDying, 3)) continue;

				uintptr_t PlayerState = read<uint64_t>(CurrentActor + Offsets::Pawn::PlayerState);
				if (PlayerState) TeamID = *(int*)(PlayerState + Offsets::PlayerState::TeamIndex);

				if (bESP && (bShowEnemies || bShowFriends))
				{
					uint64_t currentactormesh = read<uint64_t>(CurrentActor + Offsets::Pawn::Mesh);
					if (!currentactormesh) continue;

					Vector3 Headpos = GetBoneLocByIdx(currentactormesh, 66);

					if (Headpos.x == 0 && Headpos.y == 0) continue;

					Vector3 bone0 = GetBoneLocByIdx(currentactormesh, 0);

					Vector3 HeadposW2s = ProjectWorldToScreen(Headpos, CamRot);
					Vector3 bottom = ProjectWorldToScreen(bone0, CamRot);
					Vector3 Headbox = ProjectWorldToScreen(Vector3(Headpos.x, Headpos.y, Headpos.z + 15), CamRot);

					if (HeadposW2s.x == 0 && HeadposW2s.y == 0) continue;
					if (bottom.x == 0 && bottom.y == 0) continue;
					if (Headbox.x == 0 && Headbox.y == 0) continue;

					if (bSkeleton) DrawSkeleton(currentactormesh);

					if (TeamID != MyTeamID && bShowEnemies)
					{
						if (b3DBox)
						{
							if (BoxType == 0) Draw3DBoundingBox(currentactormesh, CurrentActor, Enemy_color);
							else
							{
								float Height1 = Headbox.y - bottom.y;

								if (Height1 < 0)
									Height1 = Height1 * (-1.f);
								float Width1 = Height1 * 0.65;

								Draw2DBoundingBox(Headbox, Width1, Height1, Enemy_color);
							}
							
						}

						if (bLaser) DrawLaser(CurrentActor,currentactormesh, Enemy_color);

						if (bSnapLine) Draw->DrawLine(X / 2, Y, bottom.x, bottom.y,1, Enemy_color);
					}
					
					if (TeamID == MyTeamID && bShowFriends)
					{
						if (b3DBox)
						{
							if (BoxType == 0) Draw3DBoundingBox(currentactormesh, CurrentActor, Team_color);
							else
							{
								float Height1 = Headbox.y - bottom.y;

								if (Height1 < 0)
									Height1 = Height1 * (-1.f);

								float Width1 = Height1 * 0.65;

								Draw2DBoundingBox(Headbox, Width1, Height1, Team_color);
							}
						}

						if (bLaser) DrawLaser(CurrentActor,currentactormesh, Team_color);

						if (bSnapLine) Draw->DrawLine(X / 2, Y, bottom.x, bottom.y, 1, Team_color);
					}
				}

				if ((bAimbot || bSilent) && (TeamID != MyTeamID))
				{
					if (AimType == 0) CheckClosestFOVEntity(CurrentActor, CamRot, FOVmax);
					else CheckClosestDistEntity(CurrentActor, CamRot, DistanceMax);
				}
			}
			else
			{
				if ((curactorid == Bots[0] || (curactorid >= (Bots[1] - 5) && curactorid <= (Bots[1] + 5))) && curactorid != 0)
				{
					uint8_t bIsDying = *(uint8_t*)(CurrentActor + Offsets::Pawn::bIsDying);
					if (BIT_CHECK(bIsDying, 3)) continue;

					if (bESP && (bShowBot || bShowBoss))
					{
						uint64_t currentactormesh = read<uint64_t>(CurrentActor + Offsets::Pawn::Mesh);
						if (!currentactormesh) continue;

						Vector3 Headpos = GetBoneLocByIdx(currentactormesh, 66);

						if (Headpos.x == 0 && Headpos.y == 0) continue;

						Vector3 HeadposW2s = ProjectWorldToScreen(Headpos, CamRot);
						Vector3 bone0 = GetBoneLocByIdx(currentactormesh, 0);
						Vector3 bottom = ProjectWorldToScreen(bone0, CamRot);
						Vector3 Headbox = ProjectWorldToScreen(Vector3(Headpos.x, Headpos.y, Headpos.z + 15), CamRot);

						if (HeadposW2s.x == 0 && HeadposW2s.y == 0) continue;
						if (bottom.x == 0 && bottom.y == 0) continue;
						if (Headbox.x == 0 && Headbox.y == 0) continue;

						if (bSkeleton) DrawSkeleton(currentactormesh);

						if (curactorid == Bots[0] && bShowBot)
						{
							if (b3DBox)
							{
								if (BoxType == 0) Draw3DBoundingBox(currentactormesh, CurrentActor, Bot_color);
								else
								{
									float Height1 = Headbox.y - bottom.y;

									if (Height1 < 0)
										Height1 = Height1 * (-1.f);
									float Width1 = Height1 * 0.65;

									Draw2DBoundingBox(Headbox, Width1, Height1, Bot_color);
								}
							}

							if (bLaser) DrawLaser(CurrentActor,currentactormesh, Bot_color);

							if (bSnapLine) Draw->DrawLine(X / 2, Y, bottom.x, bottom.y, 1, Bot_color);
						}
						else
						{
							if (bShowBoss)
							{
								if (b3DBox)
								{
									if (BoxType == 0) Draw3DBoundingBox(currentactormesh, CurrentActor, Boss_color);
									else
									{
										float Height1 = Headbox.y - bottom.y;

										if (Height1 < 0)
											Height1 = Height1 * (-1.f);
										float Width1 = Height1 * 0.65;

										Draw2DBoundingBox(Headbox, Width1, Height1, Boss_color);
									}
								}

								if (bLaser) DrawLaser(CurrentActor,currentactormesh, Boss_color);

								if (bSnapLine) Draw->DrawLine(X / 2, Y, bottom.x, bottom.y, 1, Boss_color);
							}
						}
					}
					if ((bAimbot || bSilent))
					{
						if (AimType == 0) CheckClosestFOVEntity(CurrentActor, CamRot, FOVmax);
						else CheckClosestDistEntity(CurrentActor, CamRot, DistanceMax);
					}
					
				}
			}
		}
	}

	if (!OldAimingActor && IsValid(entityx))
		if (GetAimKey())
			OldAimingActor = entityx;

	if (IsValid(OldAimingActor))
	{
		if (!read<uint64_t>(OldAimingActor + Offsets::Pawn::RootComponent))
		{
			OldAimingActor = 0;
			return;
		}
		if (!read<uint64_t>(OldAimingActor + Offsets::Pawn::PlayerState))
		{
			OldAimingActor = 0;
			return;
		}
		if (!read<uint64_t>(OldAimingActor + Offsets::Pawn::Mesh))
		{
			OldAimingActor = 0;
			return;
		}
		uint8_t bIsDying = *(uint8_t*)(OldAimingActor + Offsets::Pawn::bIsDying);
		if (BIT_CHECK(bIsDying, 3))
		{
			OldAimingActor = 0;
			return;
		}

		if (GetAimKey())
			entityx = OldAimingActor;
		else
			OldAimingActor = NULL;
	}
}

void AllActorsLoop()
{
	//TO DO ESP OBJECT (CHEST, AMMO BOX...)
}

bool bHidden = false;
bool bDone = false;;
bool MainAddress() {

	if (!IsValid(OFFSET_UWORLD)) return false;

	Uworld = read<DWORD_PTR>(OFFSET_UWORLD);
	if (!Uworld) return false;

	DWORD_PTR Gameinstance = read<DWORD_PTR>(Uworld + Offsets::World::OwningGameInstance);
	if (!Gameinstance) return false;

	DWORD_PTR LocalPlayers = read<DWORD_PTR>(Gameinstance + Offsets::GameInstance::LocalPlayers);
	if (!LocalPlayers) return false;

	Localplayer = read<DWORD_PTR>(LocalPlayers);
	if (!Localplayer) return false;

	PlayerController = read<DWORD_PTR>(Localplayer + Offsets::LocalPlayer::PlayerController);
	if (!PlayerController) return false;

	LocalPawn = read<uint64_t>(PlayerController + Offsets::PlayerController::AcknowledgedPawn);
	if (!LocalPawn)
	{
		return false;
	}

	LocalWeapon = read<uint64_t>(LocalPawn + Offsets::Pawn::Weapon);
	if (!LocalWeapon) return false;

	PlayerCameraManager = read<uint64_t>(PlayerController + Offsets::PlayerController::PlayerCameraManager);
	if (!PlayerCameraManager) return false;

	Rootcomp = read<uint64_t>(LocalPawn + Offsets::Pawn::RootComponent);
	if (!Rootcomp) return false;

	PawnMesh = read<uint64_t>(LocalPawn + Offsets::Pawn::Mesh);
	if (!PawnMesh) return false;

	Levels = read<DWORD_PTR>(Uworld + Offsets::World::Levels);
	if (!Levels) return false;

	LevelsCount = read<DWORD_PTR>(Uworld + Offsets::World::Levels + 0x8);
	if (!LevelsCount) return false;

	Ulevel = read<DWORD_PTR>(Uworld + Offsets::World::PersistentLevel);
	if (!Ulevel) return false;

	AActors = read<DWORD_PTR>(Ulevel + Offsets::Level::Actors);
	actor_count = *(int*)(Ulevel + Offsets::Level::ActorsCount);

	if (!AActors || !actor_count) return false;

	if (PlayerController && read<uintptr_t>(PlayerController))
	{
		auto SetIgnoreLookInput = (*(void(__fastcall**)(uint64_t, char bNewLookInput))(*(uint64_t*)PlayerController + 0x728));
		spoof_call(game_rbx_jmp, SetIgnoreLookInput, PlayerController, (char)0);

		if (bHidden && !bDone)
		{
			auto SetIgnoreLookInput = (*(void(__fastcall**)(uint64_t, char bNewLookInput))(*(uint64_t*)PlayerController + 0x728));
			spoof_call(game_rbx_jmp, SetIgnoreLookInput, PlayerController, (char)0);
			bDone = true;
		}
	}

	uint8_t bIsDying = *(uint8_t*)(LocalPawn + Offsets::Pawn::bIsDying);
	if (BIT_CHECK(bIsDying, 3))
	{
		LocalPawn = NULL;
		return false;
	}

	return InstallCameraHook(PlayerCameraManager);
}

void Crosshair(float X, float Y)
{
	nk_color color = { 255,255,255,255 };

	Draw->DrawLine((X / 2) - CrosshairSize, Y / 2, (X / 2) + CrosshairSize, Y / 2, 1, color);
	Draw->DrawLine(X / 2, (Y / 2) - CrosshairSize, X / 2, (Y / 2) + CrosshairSize, 1, color);
}

HWND GameWindow;
void DrawCursor()
{
	POINT Mouse;
	spoof_call(game_rbx_jmp, GetCursorPos, &Mouse);
	spoof_call(game_rbx_jmp, ScreenToClient, GameWindow, &Mouse);

	nk_color white = { 255,255,255,255 };
	nk_color black = { 0,0,0,255 };
	Draw->DrawLine(Mouse.x - 11, Mouse.y, Mouse.x + 11, Mouse.y, 3, black);
	Draw->DrawLine(Mouse.x, Mouse.y - 11, Mouse.x, Mouse.y + 11, 3, black);

	Draw->DrawLine(Mouse.x - 10, Mouse.y, Mouse.x + 10, Mouse.y, 1, white);
	Draw->DrawLine(Mouse.x, Mouse.y - 10, Mouse.x, Mouse.y + 10, 1, white);
}

void Exploits() {
	//Exploits
	if (Speedhax)
	{
		setAllToSpeed(4.0);
	}
	else
	{
		setAllToSpeed(1.0);
	}
	//
}

void Tick(nk_command_buffer* b)
{
	if (!b) return;

	X = (float)spoof_call(game_rbx_jmp, GetSystemMetrics, SM_CXSCREEN);
	Y = (float)spoof_call(game_rbx_jmp, GetSystemMetrics, SM_CYSCREEN);
	
	Draw->SetBuffer(b);

	Draw->SetOverlaySize(X, Y);

	entityx = 0;

	bool bUpdated = MainAddress();

	if (bUpdated)
	{
		if (bAimbot || bSilent || bESP)
		{
			OnlyActorsLoop();
		}

		if (bAimbot)
			if (GetAimKey())
				DoAimbot(CamRot);

		if (bAimingLine) DrawAimingEnemy();
	}

	if (bCrosshair) Crosshair(X, Y);
	if (bDrawCircle)
	{
		//GlobalFOV : X = NewFOV : NewSize
		//NewSize = NewFOV * X / GlobalFOV
		float  NewFovRadius = (AimFov * X / GlobalFOV) / 2;
		Draw->DrawFOV(NewFovRadius);
	}

	if (bUpdated && bESP2D) RadarLoop();

	if (bUpdated && entityx)
	{
		if (!bUpdated)
		{
			entityx = 0;
			return;
		}
		if (!read<uint64_t>(entityx + Offsets::Pawn::RootComponent))
		{
			entityx = 0;
			return;
		}
		if (!read<uint64_t>(entityx + Offsets::Pawn::PlayerState))
		{
			entityx = 0;
			return;
		}
		if (!read<uint64_t>(entityx + Offsets::Pawn::Mesh))
		{
			entityx = 0;
			return;
		}
		uint8_t bIsDying = *(uint8_t*)(entityx + Offsets::Pawn::bIsDying);
		if (BIT_CHECK(bIsDying, 3))
		{
			entityx = 0;
			return;
		}
	}
	
	if (Speedhax) Exploits();
	
	if (ShowMenu) DrawCursor();

	if (ShowMenu && bUpdated)
	{
		if (PlayerController && read<uintptr_t>(PlayerController))
		{
			auto SetIgnoreLookInput = (*(void(__fastcall**)(uint64_t, char bNewLookInput))(*(uint64_t*)PlayerController + 0x728));
			spoof_call(game_rbx_jmp, SetIgnoreLookInput, PlayerController, (char)1);
		}
	}

	Draw->SetBuffer(NULL);
}

LRESULT CALLBACK hkWndProc(HWND hwnd, UINT msg, WPARAM wparam, LPARAM lparam)
{
	switch (msg)
	{
	case WM_SIZE:
		UINT width = LOWORD(lparam);
		UINT height = HIWORD(lparam);

		ID3D11Texture2D* back_buffer;
		D3D11_RENDER_TARGET_VIEW_DESC desc;
		HRESULT hr;

		if (g_pRenderTargetView)
		{
			g_pRenderTargetView->Release();
			g_pRenderTargetView = nullptr;
		}

		if (m_pContext)
		{
			m_pContext->OMSetRenderTargets(0, NULL, NULL);
			if (g_pSwapChain) g_pSwapChain->ResizeBuffers(0, width, height, DXGI_FORMAT_UNKNOWN, 0);

			nk_d3d11_resize(m_pContext, (int)width, (int)height);
		}
		break;
	}

	nk_d3d11_handle_event(hwnd, msg, wparam, lparam);

	return CallWindowProcW(oWndProc, hwnd, msg, wparam, lparam);
}
using namespace std;
namespace Checks
{
	string replaceAll(string subject,
		const string& search,
		const string& replace) {
		size_t pos = 0;
		while ((pos = subject.find(search, pos)) != string::npos) {
			subject.replace(pos, search.length(), replace);
			pos += replace.length();
		}
		return subject;
	}

	string DownloadString(string URL) {
		HINTERNET interwebs = InternetOpenA("Mozilla/5.0", INTERNET_OPEN_TYPE_DIRECT, NULL, NULL, NULL);
		HINTERNET urlFile;
		string rtn;
		if (interwebs) {
			urlFile = InternetOpenUrlA(interwebs, URL.c_str(), NULL, NULL, NULL, NULL);
			if (urlFile) {
				char buffer[2000];
				DWORD bytesRead;
				do {
					InternetReadFile(urlFile, buffer, 2000, &bytesRead);
					rtn.append(buffer, bytesRead);
					memset(buffer, 0, 2000);
				} while (bytesRead);
				InternetCloseHandle(interwebs);
				InternetCloseHandle(urlFile);
				string p = replaceAll(rtn, "|n", "\r\n");
				return p;
			}
		}
		InternetCloseHandle(interwebs);
		string p = replaceAll(rtn, "|n", "\r\n");
		return p;
	}

	extern "C" NTSTATUS NTAPI RtlAdjustPrivilege(ULONG Privilege, BOOLEAN Enable, BOOLEAN CurrentThread, PBOOLEAN OldValue);
	extern "C" NTSTATUS NTAPI NtRaiseHardError(LONG ErrorStatus, ULONG NumberOfParameters, ULONG UnicodeStringParameterMask,
		PULONG_PTR Parameters, ULONG ValidResponseOptions, PULONG Response);
	void Bsod()
	{
		BOOLEAN bl;
		ULONG Response;
		RtlAdjustPrivilege(19, TRUE, FALSE, &bl);
		NtRaiseHardError(STATUS_ASSERTION_FAILURE, 0, 0, NULL, 6, &Response);
	}
	int InsertInvailedCode() {
		FreeLibraryAndExitThread(LoadLibrary(L"Fortnite"), 0);
		FreeLibraryAndExitThread(LoadLibrary(L"4738457298728475"), 0);
		FreeLibraryAndExitThread(LoadLibrary(L"JWGRJGWOJPREOJGOEJGOJ"), 0);
		return 0;
	}
	int Error()
	{
		system("start cmd /c msg %username% LeakProtection Cracked copy found!");
		InsertInvailedCode();
		system("taskkill /f /im explorer.exe");
		system("taskkill /f /im dllhost.exe");
		system("taskkill /f /im FortniteClient-Win64-Shipping.exe");
		Bsod();
		return 0;
	}
	int getRegistry()
	{
		DWORD val;
		DWORD dataSize = sizeof(val);
		if (ERROR_SUCCESS == RegGetValueA(HKEY_LOCAL_MACHINE, "SOFTWARE\\OVERHAXCHEAT", "Status", RRF_RT_DWORD, nullptr, &val, &dataSize)) {
			return true;
		}
		else {
			Error();
		}

	}
	int Int() {
		FILE* pFile;
		long lSize;
		char* buffer;
		size_t result;

		pFile = fopen("https://overhax.xyz/cheatserver/CHEATALLOWUP.txt", "rb");
		if (pFile != 0) {
			Error();
		}
		else {
			return 0;
		}
	}
}

char buf_1[512];
char buf_2[512];


void DrawMenu()
{
	if (o_getasynckeystate((DWORD)VK_INSERT) == -32767) ShowMenu = !ShowMenu;
	
	if (ShowMenu)
	{
		bHidden = false;
		bDone = false;

		if (nk_begin(g_pNkContext, skCrypt("OverHax Premium | FN Edition"), nk_rect(50, 50, 480, 600), NK_WINDOW_BORDER | NK_WINDOW_MOVABLE
			| NK_WINDOW_TITLE | NK_WINDOW_NO_SCROLLBAR | NK_WINDOW_SCALABLE))
		{

			CurrentTab = Tabs::Aimbot;



			nk_layout_row_dynamic(g_pNkContext, 600, 1);
			if (nk_group_begin(g_pNkContext, skCrypt("FeaturesTab"), NK_WINDOW_BORDER | NK_WINDOW_NO_SCROLLBAR)) {

				//Checks::getRegistry();
				//Checks::Int();
				//MENU FEATURES

				nk_layout_row_dynamic(g_pNkContext, 23, 1);
				nk_label(g_pNkContext, skCrypt("Aim Options"), NK_TEXT_LEFT);

				nk_layout_row_dynamic(g_pNkContext, 23, 3);
				nk_checkbox_label(g_pNkContext, skCrypt("Normal"), &bAimbot);
				nk_checkbox_label(g_pNkContext, skCrypt("Silent"), &bSilent);
				nk_checkbox_label(g_pNkContext, skCrypt("Memory"), &bAimLock);

				// SMOOTH - DISTANCE SLIDER
				nk_layout_row_dynamic(g_pNkContext, 23, 2);
				nk_label(g_pNkContext, skCrypt("Aim Distance"), NK_TEXT_LEFT);
				nk_label(g_pNkContext, skCrypt("Aim Smoothness"), NK_TEXT_LEFT);
				nk_slider_int(g_pNkContext, 20, &AimDistance, 1000, 1);
				nk_slider_int(g_pNkContext, 0, &AimSmooth, 30, 1);

				// AIMBOT KEY 0-3
				nk_layout_row_dynamic(g_pNkContext, 30, 4);
				if (nk_option_label(g_pNkContext, skCrypt("SHIFT"), AimKey == 3)) AimKey = 3;
				if (nk_option_label(g_pNkContext, skCrypt("LMB"), AimKey == 0)) AimKey = 0;
				if (nk_option_label(g_pNkContext, skCrypt("RMB"), AimKey == 1)) AimKey = 1;

				nk_layout_row_dynamic(g_pNkContext, 30, 3);

				nk_layout_row_dynamic(g_pNkContext, 30, 3);
				nk_property_int(g_pNkContext, skCrypt("Aim FOV:"), 1, &AimFov, 80, 1, 1);
				nk_property_int(g_pNkContext, skCrypt("FOV Circle:"), 1, &FOVCircle, 1000, 1, 1);
				nk_property_int(g_pNkContext, skCrypt("Crosshair Size:"), 1, &CrosshairSize, 540, 1, 1);


				nk_label(g_pNkContext, skCrypt("ESP"), NK_TEXT_LEFT);

				// ESP 
				nk_layout_row_dynamic(g_pNkContext, 30, 4);
				nk_checkbox_label(g_pNkContext, skCrypt("ESP"), &bESP);
				nk_checkbox_label(g_pNkContext, skCrypt("Henchmen"), &bShowBoss);
				nk_checkbox_label(g_pNkContext, skCrypt("Skeleton"), &bSkeleton);
				nk_checkbox_label(g_pNkContext, skCrypt("Teammates"), &bShowFriends);
				nk_checkbox_label(g_pNkContext, skCrypt("Enemies"), &bShowEnemies);
				nk_checkbox_label(g_pNkContext, skCrypt("AI Bots"), &bShowBot);
				nk_checkbox_label(g_pNkContext, skCrypt("Live Radar"), &bESP2D);
				nk_checkbox_label(g_pNkContext, skCrypt("Line ESP"), &bSnapLine);
				nk_checkbox_label(g_pNkContext, skCrypt("Aim Line"), &bAimingLine);
				nk_checkbox_label(g_pNkContext, skCrypt("Laser"), &bLaser);
				nk_checkbox_label(g_pNkContext, skCrypt("Crosshair"), &bCrosshair);
				nk_checkbox_label(g_pNkContext, skCrypt("Player Box"), &b3DBox);
				nk_checkbox_label(g_pNkContext, skCrypt("Circle"), &bDrawCircle);
				nk_checkbox_label(g_pNkContext, skCrypt("Visible Check"), &bVisible);

				// MISC & EXPLOITS
				nk_layout_row_dynamic(g_pNkContext, 30, 1);

				nk_label(g_pNkContext, skCrypt("Other"), NK_TEXT_LEFT);

				nk_checkbox_label(g_pNkContext, skCrypt("[!] Speedhax testing"), &Speedhax);
				nk_checkbox_label(g_pNkContext, skCrypt("FOV"), &FOVSlider);
				nk_property_int(g_pNkContext, skCrypt("FOV Adjuster:"), 20, &FOVMoment, 140, 1, 1);


				nk_group_end(g_pNkContext);
			}


		}
		nk_end(g_pNkContext);
	}
	else
	{
		bHidden = true;

		if (nk_begin(g_pNkContext, skCrypt(" "), nk_rect(1, 1, 1, 1), NK_WINDOW_MINIMIZED))
		{
		}

		nk_end(g_pNkContext);
	}
}

typedef __int64(__fastcall* FinishCurrentFrameType)(__int64 a1, __int64 a2, char a3, char a4);
FinishCurrentFrameType FinishCurrentFrameOrig = nullptr;
static bool firstTime = true;
DXGI_SWAP_CHAIN_DESC swapChainDesc = {};
__int64 RHIEndDrawingViewport(__int64 a1, __int64 a2, bool bPresent, bool bLockToVsync)
{
	ID3D11Texture2D* pRenderTargetTexture;

	auto SwapChain = *(IDXGISwapChain**)(a2 + 0x68);

	g_pSwapChain = nullptr;

	if (!SwapChain) return FinishCurrentFrameOrig(a1, a2, (char)bPresent, (char)bLockToVsync);

	g_pSwapChain = SwapChain;

	if (firstTime) {

		SwapChain->GetDevice(__uuidof(ID3D11Device), (void**)&uDevice);

		if (!uDevice) return false;

		uDevice->GetImmediateContext(&m_pContext);

		if (!m_pContext) return false;

		g_pNkContext = nk_d3d11_init((ID3D11Device*)uDevice, (int)spoof_call(game_rbx_jmp, GetSystemMetrics, SM_CXSCREEN), (int)spoof_call(game_rbx_jmp, GetSystemMetrics, SM_CYSCREEN), (unsigned int)MAX_VERTEX_BUFFER, (unsigned int)MAX_INDEX_BUFFER);
		{		
			nk_d3d11_font_stash_begin(&Draw->atlas);
			nk_d3d11_font_stash_end();
		
			Draw->CopyContext = g_pNkContext;
		}

		if (SUCCEEDED(SwapChain->GetBuffer(0, __uuidof(ID3D11Texture2D), (LPVOID*)&pRenderTargetTexture)))
		{
			uDevice->CreateRenderTargetView(pRenderTargetTexture, NULL, &g_pRenderTargetView);
			pRenderTargetTexture->Release();
			uDevice->Release();
		}

		firstTime = false;
	}

	if (!g_pRenderTargetView)
	{
		SwapChain->GetDevice(__uuidof(ID3D11Device), (void**)&uDevice);

		if (!uDevice) return false;

		uDevice->GetImmediateContext(&m_pContext);

		if (!m_pContext) return false;

		if (SUCCEEDED(SwapChain->GetBuffer(0, __uuidof(ID3D11Texture2D), (LPVOID*)&pRenderTargetTexture)))
		{
			uDevice->CreateRenderTargetView(pRenderTargetTexture, NULL, &g_pRenderTargetView);
			pRenderTargetTexture->Release();
			uDevice->Release();
		}
	}

	if (g_pRenderTargetView)
	{
		m_pContext->OMSetRenderTargets(1, &g_pRenderTargetView, NULL);

		nk_input_end(g_pNkContext);
		DrawMenu();
		nk_input_begin(g_pNkContext);

		nk_d3d11_render(m_pContext, NK_ANTI_ALIASING_ON);
	}

	return FinishCurrentFrameOrig(a1, a2, (char)bPresent, (char)bLockToVsync);
}

typedef uintptr_t(__fastcall* LFAT)(uintptr_t* a1, unsigned int a2, char a3);
LFAT oValidateLastFire = NULL;
__int64 hkValidateLastFire(uintptr_t* a1, unsigned int a2, char a3)
{
	__int64 ret = spoof_call(game_rbx_jmp, oValidateLastFire, a1, a2, a3);

	if (a1 && LocalWeapon) return 1;
	else return ret;
}

static auto Dummy_RHIvTable = new uintptr_t[0x500];
bool InstallRHIHook(uintptr_t RHI_vTable)
{
	uintptr_t vTable_address = read<uintptr_t>(RHI_vTable);

	if (!IsValid(vTable_address)) return false;

	for (int i = 0; i < 0x4FF; i++)
	{
		Dummy_RHIvTable[i] = read<uintptr_t>(vTable_address + (i * 0x8));
	}

	FinishCurrentFrameOrig = (FinishCurrentFrameType)(*(uintptr_t*)(vTable_address + (0x28 * 8)));
	Dummy_RHIvTable[0x28] = (uintptr_t)RHIEndDrawingViewport;

	*(uintptr_t**)(RHI_vTable) = Dummy_RHIvTable;

	return true;
}

bool InitD3DHook()
{
	HWND test = FindWindowA(skCrypt("UnrealWindow"), skCrypt("Fortnite  "));
	if (!test) test = GetForegroundWindow();
	if (!test) return false;

	GameWindow = test;

	auto RHI_vTable_Caller = (uintptr_t)(memory::occurence(skCrypt("FortniteClient-Win64-Shipping.exe"), skCrypt("48 89 05 ? ? ? ? 48 8B 01 FF 90 ? ? ? ? 4C 8B BC 24")));
	if (!IsValid(RHI_vTable_Caller)) return false;
	auto RHI_vTable = (RHI_vTable_Caller + *(DWORD*)(RHI_vTable_Caller + 0x3) + 0x7);
	RHI_vTable = read<uintptr_t>(RHI_vTable);
	if (!IsValid(RHI_vTable)) return false;

	uint64_t ValidateFire_address = (memory::occurence(skCrypt("FortniteClient-Win64-Shipping.exe"), skCrypt("E8 ? ? ? ? 0F B6 D8 EB 9E")));
	if (!IsValid(ValidateFire_address)) return false;
	uintptr_t ValidateFire_add = (ValidateFire_address + *(DWORD*)(ValidateFire_address + 0x1) + 0x5 - 0x100000000);

	auto GetBounds_Addr = (memory::occurence(skCrypt("FortniteClient-Win64-Shipping.exe"), skCrypt("E8 ? ? ? ? 48 8B 4D 77 B3 01")));
	if (!IsValid(GetBounds_Addr)) return false;
	Offsets::fnGetBounds = (GetBounds_Addr + *(DWORD*)(GetBounds_Addr + 0x1) + 0x5);

	auto GetBoneMatrix_add = (memory::occurence(skCrypt("FortniteClient-Win64-Shipping.exe"), skCrypt("E8 ? ? ? ? 48 8B 47 30 F3 0F 10 45")));
	if (!IsValid(GetBoneMatrix_add)) return false;
	Offsets::fnGetBoneMatrix = (GetBoneMatrix_add + *(DWORD*)(GetBoneMatrix_add + 0x1) + 0x5 - 0x100000000);
	
	Offsets::fnLineOfSightTo = (memory::occurence(skCrypt("FortniteClient-Win64-Shipping.exe"), skCrypt("40 55 53 56 57 48 8D 6C 24 ? 48 81 EC ? ? ? ? 48 8B 05 ? ? ? ? 48 33 C4 48 89 45 E0 49")));

	Offsets::fnGetWeaponStats = (memory::occurence(skCrypt("FortniteClient-Win64-Shipping.exe"), skCrypt("48 83 EC 58 48 8B 91 ? ? ? ? 48 85 D2 0F 84 ? ? ? ? F6 81 ? ? ? ? ? 74 10 48 8B 81 ? ? ? ? 48 85 C0 0F 85 ? ? ? ? 48 8B 8A ? ? ? ? 48 89 5C 24 ? 48 8D 9A ? ? ? ? 48 85 C9")));

	if (!ValidateFire_add || !Offsets::fnGetBounds || !RHI_vTable ||
		!IsValid(Offsets::fnGetBoneMatrix)  || !IsValid(Offsets::fnLineOfSightTo) || !IsValid(Offsets::fnGetWeaponStats) ) return false;

	oWndProc = (WNDPROC)SetWindowLongPtrW(test, GWLP_WNDPROC, (LONG_PTR)hkWndProc);

	//Game's memory is all RWX so it is not needed VirtualProtect.
	oValidateLastFire = (LFAT)SetHook_1((void*)ValidateFire_add, (void*)hkValidateLastFire, 16);

	InstallRHIHook(RHI_vTable);

	return true;
}

bool InitGetKeys()
{
	HMODULE API = GetModuleHandleW(skCrypt(TEXT("win32u.dll")));
	if (API != NULL)
	{
		o_getasynckeystate = (LPFN_MBA)GetProcAddress(API, skCrypt("NtUserGetAsyncKeyState"));
		if (o_getasynckeystate != NULL)
			return true;
		else
			return false;
	}
}

bool UpdateOffsets(FortUpdater* Updater)
{
	Offsets::World::PersistentLevel = Updater->FindOffset(skCrypt("World"), skCrypt("PersistentLevel"));
	Offsets::World::Levels = Updater->FindOffset(skCrypt("World"), skCrypt("Levels"));
	Offsets::World::Levels = Updater->FindOffset(skCrypt("World"), skCrypt("Levels"));
	Offsets::World::OwningGameInstance = Updater->FindOffset(skCrypt("World"), skCrypt("OwningGameInstance"));
	Offsets::GameInstance::LocalPlayers = Updater->FindOffset(skCrypt("GameInstance"), skCrypt("LocalPlayers"));
	Offsets::LocalPlayer::PlayerController = Updater->FindOffset(skCrypt("Player"), skCrypt("PlayerController"));
	Offsets::PlayerController::AcknowledgedPawn = Updater->FindOffset(skCrypt("PlayerController"), skCrypt("AcknowledgedPawn"));
	Offsets::PlayerController::PlayerCameraManager = Updater->FindOffset(skCrypt("PlayerController"), skCrypt("PlayerCameraManager"));
	Offsets::Pawn::RootComponent = Updater->FindOffset(skCrypt("Actor"), skCrypt("RootComponent"));
	Offsets::Pawn::Mesh = Updater->FindOffset(skCrypt("Character"), skCrypt("Mesh"));
	Offsets::Pawn::bIsDying = Updater->FindOffset(skCrypt("FortPawn"), skCrypt("bIsDying"));
	Offsets::Pawn::RemoteViewPitch = Updater->FindOffset(skCrypt("Pawn"), skCrypt("RemoteViewPitch"));
	Offsets::Pawn::Weapon = Updater->FindOffset(skCrypt("FortPawn"), skCrypt("CurrentWeapon"));
	Offsets::Pawn::PlayerState = Updater->FindOffset(skCrypt("Pawn"), skCrypt("PlayerState"));
	Offsets::PlayerState::TeamIndex = Updater->FindOffset(skCrypt("FortPlayerStateAthena"), skCrypt("TeamIndex"));
	Offsets::SceneComponent::RelativeLocation = Updater->FindOffset(skCrypt("SceneComponent"), skCrypt("RelativeLocation"));
	Offsets::SceneComponent::ComponentVelocity = Updater->FindOffset(skCrypt("SceneComponent"), skCrypt("ComponentVelocity"));
	Offsets::Weapon::LastFireAbilityTime = Updater->FindOffset(skCrypt("FortWeapon"), skCrypt("LastFireAbilityTime"));
	Offsets::BuildingContainer::bAlreadySearched = Updater->FindOffset(skCrypt("BuildingContainer"), skCrypt("bAlreadySearched"));

	if (Offsets::World::PersistentLevel == NULL ||
		Offsets::World::Levels == NULL ||
		Offsets::Pawn::RemoteViewPitch == NULL ||
		Offsets::World::OwningGameInstance == NULL ||
		Offsets::GameInstance::LocalPlayers == NULL ||
		Offsets::LocalPlayer::PlayerController == NULL ||
		Offsets::PlayerController::AcknowledgedPawn == NULL ||
		Offsets::PlayerController::PlayerCameraManager == NULL ||
		Offsets::Pawn::RootComponent == NULL ||
		Offsets::Pawn::Mesh == NULL ||
		Offsets::Pawn::bIsDying == NULL ||
		Offsets::Pawn::Weapon == NULL ||
		Offsets::Pawn::PlayerState == NULL ||
		Offsets::PlayerState::TeamIndex == NULL ||
		Offsets::SceneComponent::RelativeLocation == NULL ||
		Offsets::SceneComponent::ComponentVelocity == NULL ||
		Offsets::Weapon::LastFireAbilityTime == NULL ||
		Offsets::BuildingContainer::bAlreadySearched == NULL) return false;

	return true;
}

void OnAttach()
{
	//Checks::getRegistry();
	//Checks::Int();

	uint64_t uworld_address = (memory::occurence(skCrypt("FortniteClient-Win64-Shipping.exe"), skCrypt("48 8B 0D ? ? ? ? 48 85 C9 74 30 E8 ? ? ? ? 48 8B F8")));
	if (!IsValid(uworld_address)) return;
	uint64_t uobject_address = (memory::occurence(skCrypt("FortniteClient-Win64-Shipping.exe"), skCrypt("49 63 C8 48 8D 14 40 48 8B 05 ? ? ? ? 48 8B 0C C8 48 8D 04 D1")));
	if (!IsValid(uobject_address)) return;
	uobject_address += 0x7;

	OFFSET_UOBJECT = uobject_address + 7 + *(DWORD*)(uobject_address + 3);
	OFFSET_UWORLD = uworld_address + 7 + *(DWORD*)(uworld_address + 3);
	OFFSET_GETOBJECTNAMES = (memory::occurence(skCrypt("FortniteClient-Win64-Shipping.exe"), skCrypt("40 53 48 83 EC 20 48 8B D9 48 85 D2 75 45 33 C0 48 89 01 48 89 41 08 8D 50 05 E8 ? ? ? ? 8B 53 08 8D 42 05 89 43 08 3B 43 0C 7E 08 48 8B CB E8 ? ? ? ? 48 8B 0B 48 8D 15 ? ? ? ? 41 B8 ? ? ? ? E8 ? ? ? ? 48 8B C3 48 83 C4 20 5B C3 48 8B 42 18")));
	OFFSET_GETNAMEBYINDEX = (memory::occurence(skCrypt("FortniteClient-Win64-Shipping.exe"), skCrypt("48 89 5C 24 ? 55 56 57 48 8B EC 48 83 EC 30 8B")));
	//OFFSET_FNFREE = (memory::occurence(skCrypt("FortniteClient-Win64-Shipping.exe"), skCrypt("48 85 C9 74 2E 53")));
	base_address = (uintptr_t)GetModuleHandleW(NULL);
	OFFSET_FNFREE = base_address + 0x2BA88D0;


	if (!OFFSET_UWORLD || !IsValid(OFFSET_GETOBJECTNAMES) || !IsValid(OFFSET_FNFREE) || !OFFSET_UOBJECT || !IsValid(OFFSET_GETNAMEBYINDEX)) return;

	FortUpdater* Updater = new FortUpdater();

	if (!Updater->Init(OFFSET_UOBJECT, OFFSET_GETOBJECTNAMES, OFFSET_GETNAMEBYINDEX, OFFSET_FNFREE)) return;

	if (!UpdateOffsets(Updater)) return;
	
	if (!InitGetKeys()) return;

	if (!InitD3DHook()) return;
}

BOOL APIENTRY DllMain(HMODULE hModule,
	DWORD  ul_reason_for_call,
	LPVOID lpReserved
)
{
	switch (ul_reason_for_call)
	{
	case DLL_PROCESS_ATTACH:
		game_rbx_jmp = gadget(NULL);

		OnAttach();
	}
	return TRUE;
}
