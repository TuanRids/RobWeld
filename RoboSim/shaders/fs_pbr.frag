#version 450 core

out vec4 FragColor;
in vec3 WorldPos;
in vec3 Normal;
in vec3 FragNormal;

struct material {
    vec3 mColor;
    float mMetallic;
    float mRoughness;
    float mAo;
};
uniform material materialData;

// lights
uniform int LightModes; // 0 = single lights, 1 = world box 8 lights, 2 = 32 spherical lights, 3 = no lights, 4 = detect light
uniform vec3 lightPosition;
uniform vec3 lightColor;

uniform vec3 camPos;

const float PI = 3.14159265359;

float DistributionGGX(vec3 N, vec3 H, float roughness)
{
    float a = roughness * roughness;
    float a2 = a * a;
    float NdotH = max(dot(N, H), 0.0);
    float NdotH2 = NdotH * NdotH;

    float nom = a2;
    float denom = (NdotH2 * (a2 - 1.0) + 1.0);
    denom = PI * denom * denom;

    return nom / max(denom, 0.0000001); // prevent divide by zero for roughness=0.0 and NdotH=1.0
}

float GeometrySchlickGGX(float NdotV, float roughness)
{
    float r = (roughness + 1.0);
    float k = (r * r) / 8.0;

    float nom = NdotV;
    float denom = NdotV * (1.0 - k) + k;

    return nom / denom;
}

float GeometrySmith(vec3 N, vec3 V, vec3 L, float roughness)
{
    float NdotV = max(dot(N, V), 0.0);
    float NdotL = max(dot(N, L), 0.0);
    float ggx2 = GeometrySchlickGGX(NdotV, roughness);
    float ggx1 = GeometrySchlickGGX(NdotL, roughness);

    return ggx1 * ggx2;
}

vec3 fresnelSchlick(float cosTheta, vec3 F0)
{
    return F0 + (1.0 - F0) * pow(max(1.0 - cosTheta, 0.0), 5.0);
}

vec3 uniformSampleSphere(float u1, float u2)
{
    float z = 1.0 - 2.0 * u1;
    float r = sqrt(max(0.0, 1.0 - z * z));
    float phi = 2.0 * PI * u2;
    float x = r * cos(phi);
    float y = r * sin(phi);
    return vec3(x, y, z);
}

mat3 rotationMatrix(vec3 angles)
{
    float cx = cos(angles.x);
    float sx = sin(angles.x);
    float cy = cos(angles.y);
    float sy = sin(angles.y);
    float cz = cos(angles.z);
    float sz = sin(angles.z);

    mat3 rotX = mat3(
        1.0, 0.0, 0.0,
        0.0, cx, -sx,
        0.0, sx, cx
    );

    mat3 rotY = mat3(
        cy, 0.0, sy,
        0.0, 1.0, 0.0,
        -sy, 0.0, cy
    );

    mat3 rotZ = mat3(
        cz, -sz, 0.0,
        sz, cz, 0.0,
        0.0, 0.0, 1.0
    );

    return rotZ * rotY * rotX;
}

void main()
{
    vec3 N = normalize(Normal);
    vec3 V = normalize(camPos - WorldPos);
    vec3 albedo = materialData.mColor;
    float metallic = materialData.mMetallic;
    float roughness = materialData.mRoughness;
    float ao = materialData.mAo;

    vec3 F0 = vec3(0.04);
    F0 = mix(F0, albedo, metallic);

    vec3 ambient = vec3(0.03) * albedo * ao;
    vec3 Lo = vec3(0.0);

    if (LightModes == 1)
    {
        int lipa = 1000;
        int lipb = -1000;
        vec3 lightPositions[8] = vec3[](
            vec3(lipa, lipa, lipa),
            vec3(lipa, lipa, lipb),
            vec3(lipb, lipa, lipa),
            vec3(lipb, lipa, lipb),
            vec3(lipa, lipb, lipa),
            vec3(lipa, lipb, lipb),
            vec3(lipb, lipb, lipa),
            vec3(lipb, lipb, lipb)
        );

        for (int i = 0; i < 8; ++i)
        {
            vec3 L = normalize(lightPositions[i] - WorldPos);
            vec3 H = normalize(V + L);
            float distance = length(lightPositions[i] - WorldPos);
            float attenuation = 1.0 / (distance * distance);
            vec3 radiance = lightColor * attenuation * 100;

            float NDF = DistributionGGX(N, H, roughness);
            float G = GeometrySmith(N, V, L, roughness);
            vec3 F = fresnelSchlick(clamp(dot(H, V), 0.0, 1.0), F0);

            vec3 nominator = NDF * G * F;
            float denominator = 4 * max(dot(N, V), 0.0) * max(dot(N, L), 0.0);
            vec3 specular = nominator / max(denominator, 0.001); 

            vec3 kS = F;
            vec3 kD = vec3(1.0) - kS;
            kD *= 1.0 - metallic;

            float NdotL = max(dot(N, L), 0.0);
            Lo += (kD * albedo / PI + specular) * radiance * NdotL;
        }
    }
    else if (LightModes == 2)
    {
        vec3 lightPositions[32];
        for (int i = 0; i < 32; ++i)
        {
            float u1 = float(i) / 32.0;
            float u2 = float(i + 1) / 32.0;
            lightPositions[i] = 1000.0 * uniformSampleSphere(u1, u2);
        }

        mat3 rotation = rotationMatrix(lightPosition);

        for (int i = 0; i < 32; ++i)
        {
            vec3 L = normalize(rotation * lightPositions[i] - WorldPos);
            vec3 H = normalize(V + L);

            float distance = length(rotation * lightPositions[i] - WorldPos);
            float attenuation = 1.0 / (distance * distance);
            vec3 radiance = lightColor * attenuation * 100;

            float NDF = DistributionGGX(N, H, roughness);
            float G = GeometrySmith(N, V, L, roughness);
            vec3 F = fresnelSchlick(max(dot(H, V), 0.5), F0);

            vec3 specular = (NDF * G * F) / max(4.0 * max(dot(N, V), 0.0) * max(dot(N, L), 0.0), 0.001);
            vec3 kD = (1.0 - F) * (1.0 - metallic);

            float NdotL = max(dot(N, L) + 0.0001, 0.0);
            Lo += (kD * albedo / PI + specular) * radiance * NdotL;
        }

    }
    else if (LightModes == 3)
    {
        // Normalize the normal vector to ensure it has unit length
        vec3 normal = normalize(FragNormal);
    
        // Map the normal vector components (-1, 1) to color range (0, 1)
        vec3 color = (normal * 0.5) + 0.5;
    
        FragColor = vec4(color, 1.0);
        return;

    }
    else if (LightModes == 4)
    {
        // Normalize the normal vector to ensure it has unit length
        vec3 normal = normalize(FragNormal);

        // Define the vertical reference vector (assuming Z is up)
        vec3 vertical = vec3(0.0, 0.0, 1.0);

        // Calculate the dot product between the normal and the vertical vector
        float dotProduct = dot(normal, vertical);

        // Calculate the angle in degrees
        float angle = degrees(acos(dotProduct));

        vec3 color;
        if (angle < 10.0) { // If the angle is less than 10 degrees
            color = vec3(0.6); // Gray color for near-vertical normals
        } else {    
            color = (normal * 0.5) + 0.5; 
        }

        FragColor = vec4(color, 1.0);
        return;

    }
    else
    {
        vec3 L = normalize(lightPosition - WorldPos);
        vec3 H = normalize(V + L);
        float distance = length(lightPosition - WorldPos);
        float attenuation = 1.0 / (distance * distance);
        vec3 radiance = lightColor * attenuation;

        float NDF = DistributionGGX(N, H, roughness);
        float G = GeometrySmith(N, V, L, roughness);
        vec3 F = fresnelSchlick(clamp(dot(H, V), 0.0, 1.0), F0);

        vec3 nominator = NDF * G * F;
        float denominator = 4 * max(dot(N, V), 0.0) * max(dot(N, L), 0.0);
        vec3 specular = nominator / max(denominator, 0.001); 

        vec3 kS = F;
        vec3 kD = vec3(1.0) - kS;
        kD *= 1.0 - metallic;

        float NdotL = max(dot(N, L), 0.0);
        Lo += (kD * albedo / PI + specular) * radiance * NdotL;
    }

    // Apply tone mapping and gamma correction
    vec3 color = ambient + Lo;
    color = color / (color + vec3(1.0));
    color = pow(color, vec3(1.0 / 2.2));

    FragColor = vec4(color, 1.0); // Final color
}
