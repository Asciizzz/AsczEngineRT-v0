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
    // Copy d_framebuffer to h_framebuffer
    cudaMemcpy(h_framebuffer, d_framebuffer, width * height * sizeof(unsigned int), cudaMemcpyDeviceToHost);
    StretchDIBits(hdc, 0, 0, width, height, 0, 0, width, height, h_framebuffer, &bmi, DIB_RGB_COLORS, SRCCOPY);
}


// Clear everything
void AsczWin::Terminate() {
    delete[] h_framebuffer;
    cudaFree(d_framebuffer);
    ReleaseDC(hwnd, hdc);
    DestroyWindow(hwnd);
    UnregisterClass(L"Win32App", GetModuleHandle(nullptr));
    FreeConsole();
}


// 📦 Static Window Procedure
LRESULT CALLBACK AsczWin::WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam) {
    AsczWin* self = nullptr;
    if (uMsg == WM_NCCREATE) {
        self = static_cast<AsczWin*>(((CREATESTRUCT*)lParam)->lpCreateParams);
        SetWindowLongPtr(hwnd, GWLP_USERDATA, (LONG_PTR)self);
    } else {
        self = (AsczWin*)GetWindowLongPtr(hwnd, GWLP_USERDATA);
    }

    if (self) {
        switch (uMsg) {
            case WM_DESTROY: PostQuitMessage(0); return 0;

            // Mouse input
            case WM_MOUSEMOVE:
                self->mousePos.x = LOWORD(lParam);
                self->mousePos.y = HIWORD(lParam);
                return 0;

            case WM_LBUTTONDOWN: self->leftMouseDown = true; return 0;
            case WM_LBUTTONUP: self->leftMouseDown = false; return 0;
            case WM_RBUTTONDOWN: self->rightMouseDown = true; return 0;
            case WM_RBUTTONUP: self->rightMouseDown = false; return 0;
            
            // Keyboard input
            case WM_KEYDOWN: self->keys[wParam] = true; return 0;
            case WM_KEYUP: self->keys[wParam] = false; return 0;
        }
    }

    return DefWindowProc(hwnd, uMsg, wParam, lParam);
}