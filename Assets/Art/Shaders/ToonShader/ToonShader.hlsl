#ifndef TOONSHADER_HLSL_INCLUDED
#define TOONSHADER_HLSL_INCLUDED

#ifndef SHADERGRAPH_PREVIEW
#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Lighting.hlsl"
#endif

#define MAX_CASCADE_COUNT 20

struct SurfaceInfo
{
	float3 normalWS;
	float3 positionWS;
	float3 viewDirWS;
	bool   enableShadow;
	float3 shadowColor;
	int    cascadeCount;
	float  cascadeEnd;
	float  cascadeSpan;
	bool   enableSpecular;
	float3 specularColor;
	float  specularThreshold;
	bool   enableRim;
	float3 rimColor;
	float  rimThreshold;
};

SurfaceInfo CreateSurfaceInfo(in float3 normalWS, in float3        positionWS, in float3 viewDirWS, in bool   enableShadow, in float3   shadowColor,
                              in int    cascadeCount, in float     cascadeEnd, in float  cascadeSpan, in bool enableSpecular, in float3 specularColor,
                              in float  specularThreshold, in bool enableRim, in float3  rimColor, in float   rimThreshold)
{
	SurfaceInfo s;
	s.normalWS = normalize(normalWS);
	s.positionWS = positionWS;
	s.viewDirWS = normalize(viewDirWS);
	s.enableShadow = enableShadow;
	s.shadowColor = shadowColor;
	s.cascadeCount = clamp(cascadeCount, 1, MAX_CASCADE_COUNT);
	s.cascadeEnd = cascadeEnd;
	s.cascadeSpan = cascadeSpan;
	s.enableSpecular = enableSpecular;
	s.specularColor = specularColor;
	s.specularThreshold = specularThreshold;
	s.enableRim = enableRim;
	s.rimColor = rimColor;
	s.rimThreshold = rimThreshold;
	return s;
}

void SetShadowCascade(in int    cascadeCount, in float cascadeEnd, in float cascadeSpan, out float thresh[MAX_CASCADE_COUNT],
                      out float multiplier[MAX_CASCADE_COUNT])
{
	if(cascadeCount == 1)
	{
		thresh[0] = cascadeEnd;
		multiplier[0] = 1 + cascadeSpan;
		for(int i = 1; i < MAX_CASCADE_COUNT; i++)
		{
			thresh[i] = 0;
			multiplier[i] = 0;
		}
	}
	else
	{
		thresh[0] = 1e-6;
		multiplier[0] = 1;
		for(int i = 1; i < MAX_CASCADE_COUNT; i++)
		{
			if(i < cascadeCount)
			{
				thresh[i] = (float)i / (cascadeCount - 1) * cascadeEnd;
				multiplier[i] = 1 + cascadeSpan * i;
			}
			else
			{
				thresh[i] = 0;
				multiplier[i] = 0;
			}
		}
	}
}

#ifndef SHADERGRAPH_PREVIEW
float3 GetSingleLightCelShade(in SurfaceInfo s, in Light l)
{
	float  attenuation = l.shadowAttenuation * l.distanceAttenuation;
	float  nDotL = dot(s.normalWS, normalize(l.direction));
	float3 diffuse = 1;
	if(s.enableShadow)
	{
		diffuse = saturate(nDotL) * attenuation * l.color;
		float diffThresh[MAX_CASCADE_COUNT], diffMultiplier[MAX_CASCADE_COUNT];
		SetShadowCascade(s.cascadeCount, s.cascadeEnd, s.cascadeSpan, diffThresh, diffMultiplier);
		for(int i = 0; i < s.cascadeCount; i++)
		{
			float t = diffThresh[i], m = diffMultiplier[i];
			float mask = step(t, diffuse);
			if(mask < 1e-6)
			{
				diffuse = saturate(s.shadowColor * m);
				break;
			}
			if(i == s.cascadeCount - 1)
			{
				diffuse = 1;
			}
		}
	}
	float3 specular = 0;
	if(s.enableSpecular)
	{
		specular = saturate(dot(s.normalWS, normalize(normalize(l.direction) + s.viewDirWS))) * diffuse;
		specular = step(s.specularThreshold, specular) * s.specularColor;
	}
	float3 rim = 0;
	if(s.enableRim)
	{
		rim = (1 - saturate(dot(s.normalWS, s.viewDirWS))) * diffuse * (nDotL + 1) * 0.5;
		rim = step(s.rimThreshold, rim) * s.rimColor;
	}
	float3 finalColor = diffuse + max(specular, rim);
	return finalColor;
}
#endif

void ToonShader_float(in float3  normalWS, in float3     positionWS, in float3 viewDirWS,
                      in bool    enableShadow, in float3 shadowColor, in int cascadeCount, in float cascadeEnd, in float cascadeSpan, in bool enableSpecular,
                      in float3  specularColor, in float specularThreshold, in bool enableRim, in float3 rimColor, in float rimThreshold,
                      out float3 finalColors)
{
	finalColors = 1;
	#ifndef SHADERGRAPH_PREVIEW
	SurfaceInfo s = CreateSurfaceInfo(normalWS, positionWS, viewDirWS, enableShadow, shadowColor, cascadeCount, cascadeEnd, cascadeSpan, enableSpecular,
	                                  specularColor, specularThreshold, enableRim, rimColor, rimThreshold);
	finalColors = GetSingleLightCelShade(s, GetMainLight(TransformWorldToShadowCoord(positionWS)));
	float additionalLightsCount = GetAdditionalLightsCount();
	if(additionalLightsCount > 0)
	{
		for(int i = 0; i < additionalLightsCount; i++)
		{
			finalColors += GetSingleLightCelShade(s, GetAdditionalLight(i, positionWS));
		}
	}
	#endif
}

#endif
