#include <AsczWin.cuh>
#include <cuda_runtime.h>

// Constructor
AsczWin::AsczWin(int w, int h, std::wstring t) : width(w), height(h), title(t) {
    InitConsole();
    InitWindow();
    InitGDI();

    blockCount = (width * height + threadCount - 1) / threadCount;

    h_framebuffer = new unsigned int[width * height];
    cudaMalloc(&d_framebuffer, width * height * sizeof(unsigned int));
}

void AsczWin::InitConsole() {
    AllocConsole();
    freopen("CONOUT$", "w", stdout);
    freopen("CONOUT$", "w", stderr);
    freopen("CONIN$", "r", stdin);
    std::cout << "[Debug Console] Initialized!\n";
}

void AsczWin::InitWindow() {
    WNDCLASS wc = { 0 };
    wc.lpfnWndProc = WindowProc;
    wc.hInstance = GetModuleHandle(nullptr);
    wc.lpszClassName = L"Win32App";
    wc.cbWndExtra = sizeof(AsczWin*);  // Store pointer to our Window instance
    RegisterClass(&wc);

    hwnd = CreateWindowEx(
        0, L"Win32App", title.c_str(),
        WS_OVERLAPPEDWINDOW | WS_VISIBLE,
        CW_USEDEFAULT, CW_USEDEFAULT, width, height, nullptr, nullptr, GetModuleHandle(nullptr), this
    );

    if (!hwnd) {
        std::cerr << "Failed to create window!\n";
        exit(1);
    }

    hdc = GetDC(hwnd);
}

void AsczWin::InitGDI() {
    bmi = {};
    bmi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    bmi.bmiHeader.biWidth = width;
    bmi.bmiHeader.biHeight = -height;  // Negative to ensure top-down DIB
    bmi.bmiHeader.biPlanes = 1;
    bmi.bmiHeader.biBitCount = 32;
    bmi.bmiHeader.biCompression = BI_RGB;
}


// Draw Framebuffer to Window
void AsczWin::Draw() {
    StretchDIBits(hdc, 0, 0, width, height, 0, 0, width, height, h_framebuffer, &bmi, DIB_RGB_COLORS, SRCCOPY);
}