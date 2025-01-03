#include <MatRtxFlags.cuh>

MatRtxFlags::MatRtxFlags() :
    reflection(false), refraction(false), shadow(true),
    emission(false), caustics(false), fresnel(false),
    trnsprncy(false), specular(false)
{}

MatRtxFlags::MatRtxFlags(
    bool reflection, bool refraction, bool shadow, bool emission, bool caustics,
    bool fresnel, bool trnsprncy, bool specular
) :
    reflection(reflection), refraction(refraction),
    shadow(shadow), emission(emission), caustics(caustics),
    fresnel(fresnel), trnsprncy(trnsprncy), specular(specular)
{}

MatRtxFlags::MatRtxFlags(const MatAttrs& attrs, const MatFlags& flags, int illum = 0) {
    // Initialize flags based on the illumination model and attributes
    determineFlags(attrs, flags, illum);
}

void MatRtxFlags::determineFlags(const MatAttrs& attrs, const MatFlags& flags, int illum) {
    // Base decisions on the illumination model
    switch (illum) {
        case 0: // Color on, Ambient off
            shadow = false;
            break;
        case 1: // Color on, Ambient on
            shadow = true;
            break;
        case 2: // Highlight on (Specular)
            shadow = true;
            specular = flags.has_ks && (attrs.ks_r > 0.0f || attrs.ks_g > 0.0f || attrs.ks_b > 0.0f);
            break;
        case 3: // Reflection on and Ray trace on
            reflection = flags.has_kr && (attrs.kr_r > 0.0f || attrs.kr_g > 0.0f || attrs.kr_b > 0.0f);
            shadow = true;
            break;
        case 4: // trnsprncy: Glass on, Reflection: Ray trace on
            trnsprncy = flags.has_trnsprncy && attrs.trnsprncy > 0.0f && attrs.trnsprncy < 1.0f;
            reflection = flags.has_kr;
            shadow = true;
            break;
        case 5: // Reflection: Fresnel on and Ray trace on
            reflection = flags.has_kr;
            fresnel = flags.has_kr; // Fresnel requires reflection properties
            shadow = true;
            break;
        case 6: // trnsprncy: Refraction on, Fresnel off and Ray trace on
            refraction = flags.has_ior && attrs.ior > 1.0f;
            trnsprncy = flags.has_trnsprncy;
            shadow = true;
            break;
        case 7: // trnsprncy: Refraction on, Fresnel on and Ray trace on
            refraction = flags.has_ior && attrs.ior > 1.0f;
            trnsprncy = flags.has_trnsprncy;
            fresnel = flags.has_kr; // Refraction + Fresnel
            shadow = true;
            break;
        case 8: // Reflection on and Ray trace off
            reflection = flags.has_kr;
            shadow = true;
            break;
        case 9: // trnsprncy: Glass on, Reflection: Ray trace off
            trnsprncy = flags.has_trnsprncy;
            reflection = false; // No ray tracing for reflection
            shadow = true;
            break;
        case 10: // Casts Shadows onto Invisible Surfaces
            shadow = true;
            break;
    }

    // General checks for emission and caustics
    emission = flags.has_ke && 
                        (attrs.ke_r > 0.0f || attrs.ke_g > 0.0f || attrs.ke_b > 0.0f);
    caustics = reflection || refraction;
}