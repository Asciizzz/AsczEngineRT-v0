#include <AsczWin.cuh>
#include <cuda_runtime.h>

#include <string>
#include <sstream>
#include <iomanip>

__global__ void copyToDrawBuffer(Flt3 *frmbuffer, unsigned int *drawbuffer, int width, int height) {
    int tIdx = blockIdx.x * blockDim.x + threadIdx.x;
    if (tIdx < width * height) {
        int x = tIdx % width;
        int y = tIdx / width;
        int i = y * width + x;
        Flt3 color = frmbuffer[i];
        drawbuffer[i] = (int(color.x * 255) << 16) | (int(color.y * 255) << 8) | int(color.z * 255);
    }
}

// Constructor
AsczWin::AsczWin(int w, int h, std::wstring t) : width(w), height(h), title(t) {
    InitConsole();
    InitWindow();
    InitGDI();

    blockCount = (width * height + threadCount - 1) / threadCount;

    cudaMalloc(&d_frmbuffer, width * height * sizeof(Flt3));
    cudaMalloc(&d_drawbuffer, width * height * sizeof(unsigned int));
    h_drawbuffer = new unsigned int[width * height];
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


// Debug
void AsczWin::DrawText(HDC hdc, int x, int y, const AsczDebug &db) {
    SetBkMode(hdc, TRANSPARENT);
    SetTextColor(hdc, RGB(db.color.x, db.color.y, db.color.z));
    TextOut(hdc, x, y, db.text.c_str(), db.text.length());
}
void AsczWin::appendDebug(std::wstring text, Int3 color) {
    AsczDebug db;
    db.text = text;
    db.color = color;
    debugs.push_back(db);
}
void AsczWin::appendDebug(std::string text, Int3 color) {
    appendDebug(std::wstring(text.begin(), text.end()), color);
}

// Framebuffer
void AsczWin::DrawFramebuffer() {
    copyToDrawBuffer<<<blockCount, threadCount>>>(d_frmbuffer, d_drawbuffer, width, height);
    cudaDeviceSynchronize();

    cudaMemcpy(h_drawbuffer, d_drawbuffer, width * height * sizeof(unsigned int), cudaMemcpyDeviceToHost);
    StretchDIBits(hdc, 0, 0, width, height, 0, 0, width, height, h_drawbuffer, &bmi, DIB_RGB_COLORS, SRCCOPY);
}

// Draw everything
void AsczWin::Draw() {
    DrawFramebuffer();

    for (int i = 0; i < debugs.size(); i++) {
        DrawText(hdc, 10, 10 + i * 20, debugs[i]);
    }
    debugs.clear();
}


// Clear everything
void AsczWin::Terminate() {
    cudaFree(d_frmbuffer);
    cudaFree(d_drawbuffer);
    delete[] h_drawbuffer;

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