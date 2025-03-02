#ifndef ASCZWIN_CUH
#define ASCZWIN_CUH

#define WIN32_LEAN_AND_MEAN  // Reduce Windows header bloat

#define UNICODE
#define _UNICODE

#include <Vector.cuh>
#include <windows.h>

struct AsczDebug {
    std::wstring text;
    Int3 color;
};

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


    // Constructor
    AsczWin(int w, int h, std::wstring t);
    void InitWindow();
    void InitGDI();

    // Debug
    std::vector<AsczDebug> debugs;

    void DrawText(HDC hdc, int x, int y, const AsczDebug &db);
    void appendDebug(std::wstring text, Int3 color=255);
    void appendDebug(std::string text, Int3 color=255);

    // Framebuffers
    int threadCount = 256;
    int blockCount;
    unsigned int* h_framebuffer;
    unsigned int* d_framebuffer;
    void DrawFramebuffer();


    // Draw
    void Draw();

    // Destroy Window
    void Terminate();

    // 📦 Static Window Procedure
    static LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam);
};

#endif