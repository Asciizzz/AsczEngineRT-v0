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

    // Constructor
    AsczWin(int w, int h, std::wstring t);
    void InitConsole();
    void InitWindow();
    void InitGDI();

    // Draw Framebuffer
    void Draw();
    // Destroy Window
    void Terminate();

    // 📦 Static Window Procedure
    static LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam);
};

#endif