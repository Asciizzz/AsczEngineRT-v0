#include <Material.cuh>

Material::Material() : name(nullptr) {}
Material::Material(const char *name, const MatAttrs& attrs, const MatFlags& flags, const MatRtxFlags& rtxFlags) :
    name(name), attrs(attrs), flags(flags), rtxFlags(rtxFlags)
{}

Material::Material(const char *name, const MatAttrs& attrs, const MatFlags& flags) :
    name(name), attrs(attrs), flags(flags)
{
    determineRtxFlags(attrs, flags);
}

void Material::determineRtxFlags(const MatAttrs& attrs, const MatFlags& flags) {
    // Base decisions on the illumination model
    switch (attrs.illum_mdl) {
        case 0: // Color on, Ambient off
            rtxFlags.shadow = false;
            break;
        case 1: // Color on, Ambient on
            rtxFlags.shadow = true;
            break;
        case 2: // Highlight on (Specular)
            rtxFlags.shadow = true;
            rtxFlags.specular = flags.has_ks && (attrs.ks_r > 0.0f || attrs.ks_g > 0.0f || attrs.ks_b > 0.0f);
            break;
        case 3: // Reflection on and Ray trace on
            rtxFlags.reflection = flags.has_kr && (attrs.kr_r > 0.0f || attrs.kr_g > 0.0f || attrs.kr_b > 0.0f);
            rtxFlags.shadow = true;
            break;
        case 4: // trnsprncy: Glass on, Reflection: Ray trace on
            rtxFlags.trnsprncy = flags.has_trnsprncy && attrs.trnsprncy > 0.0f && attrs.trnsprncy < 1.0f;
            rtxFlags.reflection = flags.has_kr;
            rtxFlags.shadow = true;
            break;
        case 5: // Reflection: Fresnel on and Ray trace on
            rtxFlags.reflection = flags.has_kr;
            rtxFlags.fresnel = flags.has_kr; // Fresnel requires reflection properties
            rtxFlags.shadow = true;
            break;
        case 6: // trnsprncy: Refraction on, Fresnel off and Ray trace on
            rtxFlags.refraction = flags.has_ior && attrs.ior > 1.0f;
            rtxFlags.trnsprncy = flags.has_trnsprncy;
            rtxFlags.shadow = true;
            break;
        case 7: // trnsprncy: Refraction on, Fresnel on and Ray trace on
            rtxFlags.refraction = flags.has_ior && attrs.ior > 1.0f;
            rtxFlags.trnsprncy = flags.has_trnsprncy;
            rtxFlags.fresnel = flags.has_kr; // Refraction + Fresnel
            rtxFlags.shadow = true;
            break;
        case 8: // Reflection on and Ray trace off
            rtxFlags.reflection = flags.has_kr;
            rtxFlags.shadow = true;
            break;
        case 9: // trnsprncy: Glass on, Reflection: Ray trace off
            rtxFlags.trnsprncy = flags.has_trnsprncy;
            rtxFlags.reflection = false; // No ray tracing for reflection
            rtxFlags.shadow = true;
            break;
        case 10: // Casts Shadows onto Invisible Surfaces
            rtxFlags.shadow = true;
            break;
    }

    // General checks for emission and caustics
    rtxFlags.emission = flags.has_ke && 
                        (attrs.ke_r > 0.0f || attrs.ke_g > 0.0f || attrs.ke_b > 0.0f);
    rtxFlags.caustics = rtxFlags.reflection || rtxFlags.refraction;
}