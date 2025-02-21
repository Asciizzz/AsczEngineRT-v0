#ifndef ASCZWIN_CUH
#define ASCZWIN_CUH

#define WIN32_LEAN_AND_MEAN  // Reduce Windows header bloat

#include <Vector.cuh>
#include <windows.h>

class AsczWin {
public:
    int width, height;
    std::wstring title;
    HWND hwnd;
    HDC hdc;
    BITMAPINFO bmi;
    POINT mousePos = { 0, 0 };
    bool leftMouseDown = false;
    bool rightMouseDown = false;
    bool keys[256] = { false };

    unsigned int* h_framebuffer;
    unsigned int* d_framebuffer;

    // 🎯 Constructor
    AsczWin(int w, int h, std::wstring t) : width(w), height(h), title(t) {
        InitConsole();
        InitWindow();
        InitGDI();

        h_framebuffer = new unsigned int[width * height];
    }

    // 📺 Initialize the debug console
    void InitConsole() {
        AllocConsole();
        freopen("CONOUT$", "w", stdout);
        freopen("CONOUT$", "w", stderr);
        freopen("CONIN$", "r", stdin);
        std::cout << "[Debug Console] Initialized!\n";
    }

    // 📦 Create and register the window
    void InitWindow() {
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

    // 🎨 Initialize GDI for drawing
    void InitGDI() {
        bmi = {};
        bmi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
        bmi.bmiHeader.biWidth = width;
        bmi.bmiHeader.biHeight = -height;  // Negative to ensure top-down DIB
        bmi.bmiHeader.biPlanes = 1;
        bmi.bmiHeader.biBitCount = 32;
        bmi.bmiHeader.biCompression = BI_RGB;
    }

    // 🎨 Fill the framebuffer with a gradient
    void RenderGradient() {
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                unsigned char r = (x * 255) / width;
                unsigned char g = (y * 255) / height;
                unsigned char b = 128;

                // Reduce the intensity of the gradient based on the distance from the mouse
                int dx = x - mousePos.x;
                int dy = y - mousePos.y;
                float dist = sqrtf(dx * dx + dy * dy);
                float intensity = 1.0f - (dist / 200.0f);
                r = (unsigned char)(r * intensity);
                g = (unsigned char)(g * intensity);
                b = (unsigned char)(b * intensity);

                h_framebuffer[y * width + x] = (b << 16) | (g << 8) | r; // BGRA format
            }
        }
    }

    // 🎯 Draw the framebuffer to the window
    void Draw() {
        StretchDIBits(hdc, 0, 0, width, height, 0, 0, width, height,
            h_framebuffer, &bmi, DIB_RGB_COLORS, SRCCOPY);
    }

    // 🎮 Handle input and print to the debug console
    void HandleInput() {
        std::cout   << "\rMouse: (" << mousePos.x << ", " << mousePos.y << ") "
                    << "LMB: " << leftMouseDown
                    << " RMB: " << rightMouseDown
                    << " W: " << keys['W']
                    << " A: " << keys['A']
                    << " S: " << keys['S']
                    << " D: " << keys['D'] << "         " << std::flush;
    }

    // 🚀 Run the main loop
    void Run() {
        std::cout << "Rendering gradient...\n";

        MSG msg = { 0 };
        while (msg.message != WM_QUIT) {
            RenderGradient();
            if (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE)) {
                TranslateMessage(&msg);
                DispatchMessage(&msg);
            }
            Draw();
            HandleInput();
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