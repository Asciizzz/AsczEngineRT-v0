# **Material Attributes (MatAttrs) Explanation**

---

### **1. Color Attributes**

These attributes define the color and light interaction of the material. They are used to simulate how the material interacts with various light sources.

#### **[`Ka (Ambient Color)`]**
- **Description**: Ambient light color that represents the material’s appearance under low light conditions.
- **Range**: `[0.0, 1.0]` for each RGB component.
- **Example**:  
  `Ka = [0.2f, 0.1f, 0.05f];`  // Dim brown ambient light for wood
- **Material Example**:  
  **Wood**: Dark brown ambient color to simulate wood under soft light.

#### **[`Kd (Diffuse Color)`]**
- **Description**: The primary color of the material that represents how it reflects light under normal lighting conditions.
- **Range**: `[0.0, 1.0]` for each RGB component.
- **Example**:  
  `Kd = [0.6f, 0.3f, 0.1f];`  // Brown diffuse color for wood
- **Material Example**:  
  **Stone**: Light gray diffuse color for stone.

#### **[`Ks (Specular Color)`]**
- **Description**: The color of specular reflections, representing how glossy or shiny the material appears.
- **Range**: `[0.0, 1.0]` for each RGB component.
- **Example**:  
  `Ks = [0.8f, 0.8f, 0.8f];`  // White specular reflection for polished stone
- **Material Example**:  
  **Metal**: Strong white specular highlights for a shiny surface.

#### **[`Ke (Emission Color)`]**
- **Description**: Defines the emission color of the material. Emission simulates the material glowing.
- **Range**: `[0.0, 1.0]` for each RGB component.
- **Example**:  
  `Ke = [1.0f, 0.0f, 0.0f];`  // Red emission for glowing object
- **Material Example**:  
  **LED Light**: Red or white emission to simulate glowing.

---

### **2. Texture Attributes**

These attributes are used to apply textures to different parts of the material, such as ambient, diffuse, specular, and more. Texture IDs are used to link textures to the material.

- **map_Ka**: Ambient texture map
- **map_Kd**: Diffuse texture map
- **map_Ks**: Specular texture map
- **map_Ke**: Emission texture map
- **map_bump**: Bump map
- **map_displace**: Displacement map
- **map_reflect**: Reflection map

These are self-explanatory and simply link a texture to each corresponding material attribute, but will override the color attribute if both are present.

---

### **3. Reflection and Refraction Attributes**

These attributes define how light interacts with the material in terms of reflection and refraction, affecting the appearance of transparent and reflective materials.

#### **[`Ni (Index of Refraction)`]**
- **Description**: Describes how much light bends when passing through the material. Used to simulate transparent materials like glass.
- **Range**: `[1.0, 2.0]`
- **Example**:  
  `Ni = 1.5f;`  // Glass-like material
- **Material Example**:  
  **Glass**: An index of refraction around `1.5f`.

#### **[`reflectivity`]**
- **Description**: Defines the material's reflectivity, controlling how much light is reflected off its surface.
- **Range**: `[0.0, 1.0]`
- **Example**:  
  `reflectivity = 0.5f;`  // Moderate reflectivity for polished stone
- **Material Example**:  
  **Wood**: Lower reflectivity for a matte, non-reflective surface.

#### **[`Refract (Refraction Color)`]**
- **Description**: Defines the color of light that passes through transparent materials.
- **Range**: `[0.0, 1.0]` for each RGB component.
- **Example**:  
  `Refract = [0.8f, 0.9f, 1.0f];`  // Clear blue refraction for glass
- **Material Example**:  
  **Glass**: Clear blue refraction color for glass.

---

### **4. Surface Interaction Attributes**

These attributes influence how light interacts with the material’s surface, such as how shiny or rough the material appears.

#### **[`shiny`]**
- **Description**: Defines the shininess of the material. Higher values produce sharper, more concentrated specular reflections.
- **Range**: `0.0` to `128.0`
- **Example**:  
  `shiny = 100.0f;`  // High shininess for polished metal
- **Material Example**:  
  **Metal**: High shininess value for smooth, reflective metal surfaces.

#### **[`roughness`]**
- **Description**: Defines the roughness of the material’s surface, affecting the spread of specular reflections.
- **Range**: `0.0` to `1.0`
- **Example**:  
  `roughness = 0.8f;`  // Rough surface like concrete
- **Material Example**:  
  **Concrete**: High roughness to simulate a matte, rough surface.

#### **[`clearcoat`]**
- **Description**: The intensity of the clearcoat layer applied to the material to give it a glossy finish.
- **Range**: `0.0` to `1.0`
- **Example**:  
  `clearcoat = 1.0f;`  // High-gloss clearcoat for car paint
- **Material Example**:  
  **Car Paint**: A polished, high-gloss finish for a vehicle surface.

#### **[`clearcoat_gloss`]**
- **Description**: Defines the glossiness of the clearcoat, with a higher value resulting in sharper highlights.
- **Range**: `0.0` to `1.0`
- **Example**:  
  `clearcoat_gloss = 0.9f;`  // High gloss for shiny surfaces
- **Material Example**:  
  **Car Paint**: High gloss for a smooth, shiny finish.

---

### **5. Transparency Attributes**

These attributes control the transparency and transmission properties of the material.

#### **[`Tr`]**
- **Description**: Defines the transparency of the material, with `0.0` being fully opaque and `1.0` being fully transparent.
- **Range**: `0.0` to `1.0`
- **Example**:  
  `Tr = 0.5f;`  // Semi-transparent material like frosted glass
- **Material Example**:  
  **Glass**: Transparent material for frosted glass.

#### **[`transmit`]**
- **Description**: Defines the color of light transmitted through the material.
- **Range**: `[0.0, 1.0]` for each RGB component.
- **Example**:  
  `transmit = [0.0f, 0.8f, 1.0f];`  // Light blue transmission for water
- **Material Example**:  
  **Water**: A blue transmission color for realistic water.

---

### **6. Anisotropy**

#### **[`anisotropy`]**
- **Description**: Defines the anisotropy of the material, used to simulate brushed metal-like surfaces.
- **Range**: `0.0` to `1.0`
- **Example**:  
  `anisotropy = 0.7f;`  // Brushed metal effect
- **Material Example**:  
  **Brushed Steel**: Medium anisotropy for a brushed metal effect.

---

### **7. Illumination Model**

#### **[`illum`]**
- **Description**: Specifies the illumination model used for the material. A common value is `2`, which represents diffuse and specular components.
- **Range**: Integer values, with `2` being the most commonly used for diffuse + specular lighting.
- **Example**:  
  `illum = 2;`  // Diffuse + specular model
- **Material Example**:  
  **Stone**: A simple diffuse + specular illumination model for stone.

---

### **8. Other Properties**

#### **[`map_reflect`]**
- **Description**: Links a texture that simulates reflection on the material.
- **Range**: Integer value representing texture ID.

#### **[`map_displace`]**
- **Description**: Links a texture that modifies the geometry of the material by displacing vertices.
- **Range**: Integer value representing texture ID.