#ifndef ASCZWIN_CUH
#define ASCZWIN_CUH

#define WIN32_LEAN_AND_MEAN  // Reduce Windows header bloat

#define UNICODE
#define _UNICODE

#include <Vector.cuh>
#include <windows.h>

class AsczWin {
public:
    // Window properties
    int width, height;
    std::wstring title;
    HWND hwnd;
    HDC hdc;
    BITMAPINFO bmi;

    // Input state handling
    POINT mousePos = { 0, 0 };
    bool leftMouseDown = false;
    bool rightMouseDown = false;
    bool keys[256] = { false };

    // Framebuffers
    unsigned int* h_framebuffer;
    unsigned int* d_framebuffer;
    int threadCount = 256;
    int blockCount;

    AsczWin(int w, int h, std::wstring t);
    void InitConsole();
    void InitWindow();
    void InitGDI();

    void Draw();

    // 🎮 Handle input and print to the debug console
    // void HandleInput() {
    //     std::cout   << "\rMouse: (" << mousePos.x << ", " << mousePos.y << ") "
    //                 << "LMB: " << leftMouseDown
    //                 << " RMB: " << rightMouseDown
    //                 << " W: " << keys['W']
    //                 << " A: " << keys['A']
    //                 << " S: " << keys['S']
    //                 << " D: " << keys['D']
    //                 << " Space: " << keys[VK_SPACE]
    //                 << " LShift: " << keys[VK_LSHIFT]
    //                 << " LCtrl: " << keys[VK_LCONTROL]
    //                 << "         " << std::flush;
    // }

    // 🚀 Run the main loop
    void Run() {

        MSG msg = { 0 };
        while (msg.message != WM_QUIT) {
            if (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE)) {
                TranslateMessage(&msg);
                DispatchMessage(&msg);
            }

            Draw();
            Sleep(16);  // ~60 FPS
        }

        ReleaseDC(hwnd, hdc);
    }

    // 📦 Static Window Procedure
    static LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam) {
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
};

#endif