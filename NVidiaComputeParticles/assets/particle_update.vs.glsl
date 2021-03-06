//----------------------------------------------------------------------------------
// File:        ComputeParticles/assets/shaders/particlesCS.glsl
// SDK Version: v1.2
// Email:       gameworks@nvidia.com
// Site:        http://developer.nvidia.com/
//
// Copyright (c) 2014, NVIDIA CORPORATION. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//----------------------------------------------------------------------------------
#version 410

layout (std140) uniform ParticleParams
{
    uniform vec4	attractor;
    uniform float	numParticles;
    uniform float	damping;
    uniform float	noiseFreq;
    uniform float	noiseStrength;
    uniform float	invNoiseSize;
};

uniform sampler3D	noiseTex3D;

in vec4   iPosition;
in vec4   iVelocity;

out vec4  oPosition;
out vec4  oVelocity;
out vec4  oQuadPosition1;
out vec4  oQuadPosition2;
out vec4  oQuadPosition3;
out vec4  oQuadPosition4;

// noise functions
// returns random value in [-1, 1]
vec3 noise3f( vec3 p )
{
    return texture( noiseTex3D, p * invNoiseSize ).xyz;
}

// Fractal sum
vec3 fBm3f( vec3 p, int octaves, float lacunarity, float gain )
{
    float freq = 1.0;
    float amp = 0.5;
    vec3 sum = vec3( 0.0 );
    for( int i = 0; i < octaves; i++ )
    {
        sum += noise3f( p * freq ) * amp;
        freq *= lacunarity;
        amp *= gain;
    }
    return sum;
}

vec3 attract( vec3 p, vec3 p2 )
{
    const float softeningSquared = 0.01;
    vec3 v = p2 - p;
    float r2 = dot( v, v );
    r2 += softeningSquared;
    float invDist = 1.0f / sqrt( r2 );
    float invDistCubed = invDist * invDist * invDist;
    return v * invDistCubed;
}

// Compute shader to update particles
void main()
{
    // Thread block size may not be exact multiple of number of particles.
    if( gl_VertexID >= numParticles )
    {
        return;
    }
    
    // Read particle position and velocity from buffers.
    vec3 p = iPosition.xyz;
    vec3 v = iVelocity.xyz;
    
    v += fBm3f( p * noiseFreq, 4, 2.0, 0.5 ) * noiseStrength;
    v += attract( p, attractor.xyz ) * attractor.w;
    
    // Integrate
    p += v;
    v *= damping;
    
    // Write new values
    oPosition = vec4( p, 1.0 );
    oVelocity = vec4( v, 0.0 );
    oQuadPosition1 = oPosition;
    oQuadPosition2 = oPosition;
    oQuadPosition3 = oPosition;
    oQuadPosition4 = oPosition;
}

