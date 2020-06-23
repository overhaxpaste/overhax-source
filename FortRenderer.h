#include <windows.h>
#include "utils.h"

#include "Vectors.h"
#include "FW1FontWrapper.h"
#include <D3D11.h>

#include "Vectors.h"

static ID3D11DeviceContext* m_pContext;
static ID3D11RenderTargetView* g_pRenderTargetView;
static IDXGISwapChain* g_pSwapChain;

static WNDPROC oWndProc;

typedef int (WINAPI* LPFN_MBA)(DWORD);
static LPFN_MBA o_getasynckeystate; 

static HWND hWindow;
static HWND hWnd;
static int CurrentTab;

static bool ShowMenu = true;

static int bAimbot = 0; //added 
static int bAimLock = 0; //added 
static int bSilent = 0; //added 
static int bStick = 0; //added 
static int bVisible = 0; //added
static int AimKey = 3; //added
static int AimType = 0; //added
static int AimFov = 50; //added
static int AimDistance = 300; //added
static int AimSmooth = 0; //added

static int bESP = 0; //added
static int bESP2D = 0; //adedd , radio?
static int bShowEnemies = 0; //added
static int bShowFriends = 0; //added
static int bShowBot = 0; //added
static int bShowBoss = 0; //added
static int bSkeleton = 0; //added
static int b3DBox = 0;  //added, radio?
static int BoxType = 0;
static int bLaser = 0;  //added
static int bSnapLine = 0; //added
static int bAimingLine = 0; //added 

// not added yet
static int bObjectESP = 0;
static int bChests = 0;
static int bAmmoBox = 0;
static int bPickups = 0;
static int bLlama = 0;

// not added yet
static int bInstantReload = 0;
static int bNoSpread = 0;


static int bDrawCircle = 0; //added 
static int FOVCircle = 150; //added
static int bCrosshair = 0; //added 
static int CrosshairSize = 10; //added 
static int FOVSlider = 0;
static int FOVMoment = 80;
static int Speedhax = 0;

static int testint = 80;

enum Keys {
	LButton,
	RButton,
	Alt_Button,
	Shift
};

enum Tabs {
	Aimbot,
	ESP,
	Objects,
	Misc
};