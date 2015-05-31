//----------------------------------------------------------------------------------
// File:        ComputeParticles/ParticleSystem.cpp
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

#if 0

#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/Shader.h"
#include "cinder/gl/Batch.h"
#include "cinder/gl/VboMesh.h"
#include "cinder/gl/Fbo.h"
#include "cinder/Camera.h"
#include "cinder/CameraUi.h"
#include "cinder/ImageIo.h"
#include "cinder/Utilities.h"
#include "cinder/Rand.h"
#include "cinder/params/Params.h"
#include "cinder/gl/Context.h"
//#include "Ssbo.h"
//#include "ScopedBufferBase.h"

using namespace ci;
using namespace ci::app;

struct ShaderParams
{
    vec4 attractor;
    uint32_t numParticles;
    float spriteSize;
    float damping;
    float noiseFreq;
    float noiseStrength;
    
    ShaderParams() :
    spriteSize( 0.015f ),
    attractor( 0.0f, 0.0f, 0.0f, 0.0f ),
    damping( 0.95f ),
    noiseFreq( 10.0f ),
    noiseStrength( 0.001f )
    {
    }
};

struct Particle
{
    Particle()
    : pos( 0 ),
    vel( 0 )
    {
    }
    
    vec4	pos;
    vec4	vel;
};

class TransformFeedbackApp : public App {
public:
    TransformFeedbackApp();
    
    virtual void	update() override;
    virtual void	draw() override;
    //void	prepareSettings( Settings *settings ) override;
    
    void renderScene( gl::GlslProgRef effect );
    void setupShaders();
    void setupTextures();
    void setupFbos();
    void setupBuffers();
    void resetParticleSystem( float size );
    void updateParticleSystem();
    void setupNoiseTexture3D();
    
    enum { WORK_GROUP_SIZE = 128, NUM_PARTICLES = 1 << 18 };
    
    //gl::VboRef mVBO;
    //gl::VboMeshRef teapot;
    gl::GlslProgRef mRenderProg;
    gl::GlslProgRef mUpdateProg;
    
    //SsboT<vec4>::Ref mPos;
    //SsboT<vec4>::Ref mVel;
    // Descriptions of particle data layout.
    gl::VaoRef		mAttributes[2];
    // Buffers holding raw particle data on GPU.
    //gl::VboRef		mParticleBuffer[2];
    gl::VaoRef mQuadPosVao;
    
    std::uint32_t	mSourceIndex		= 0;
    std::uint32_t	mDestinationIndex	= 1;
    gl::VboRef mParticles[2];
    gl::VboRef mQuadPositions;
    //gl::VboRef mVel;
    gl::VboRef mIndicesVbo;
    
    params::InterfaceGlRef mParams;
    ShaderParams mShaderParams;
};

TransformFeedbackApp::TransformFeedbackApp()
{
    setupShaders();
    setupBuffers();
    resetParticleSystem( 0.5f );
}

void TransformFeedbackApp::setupShaders()
{
    //try {
    //    mRenderProg = gl::GlslProg::create( gl::GlslProg::Format().vertex( loadAsset( "render.vs.glsl" ) )
    //                                       .fragment( loadAsset( "render.fs.glsl" ) ) );
    //}
    //catch( gl::GlslProgCompileExc e ) {
    //    ci::app::console() << e.what() << std::endl;
    //    quit(); //???
    //}
    
    try {
        
        /*
        mUpdateProg = gl::GlslProg::create( gl::GlslProg::Format()
                                           .vertex( loadAsset( "particle_test.vs.glsl" ) )
                                           .feedbackFormat( GL_INTERLEAVED_ATTRIBS )
                                           .feedbackVaryings( { "oPosition", "oVelocity", "gl_NextBuffer", "oQuadPosition1", "oQuadPosition2", "oQuadPosition3", "oQuadPosition4" } )
                                           .attribLocation( "iPosition", 0 )
                                           //.attribLocation( "iVelocity", 1 ) // optimized out???
                                           );
         //*/
        //*
        mUpdateProg = gl::GlslProg::create( gl::GlslProg::Format()
                                           .vertex( loadAsset( "particle_test.vs.glsl" ) )
                                           .feedbackFormat( GL_INTERLEAVED_ATTRIBS )
                                           .feedbackVaryings( { "oPosition", "oVelocity" } )
                                           .attribLocation( "iPosition", 0 )
                                           //.attribLocation( "iVelocity", 1 )
                                           );
         //*/
    }
    catch( gl::GlslProgCompileExc e ) {
        ci::app::console() << e.what() << std::endl;
        quit(); //???
    }
    
    CI_CHECK_GL();
    //mUpdateProg->uniform( "invNoiseSize", 1.0f / mNoiseSize );
    //mUpdateProg->uniform( "noiseTex3D", 0 );
}

void TransformFeedbackApp::setupBuffers()
{
    std::vector<Particle> particles( NUM_PARTICLES, Particle() );
    //particles.assign(  );
    //mPos = SsboT<vec4>::create( NUM_PARTICLES, GL_STATIC_DRAW );
    //mVel = SsboT<vec4>::create( NUM_PARTICLES, GL_STATIC_DRAW );
    mParticles[0] = gl::Vbo::create( GL_ARRAY_BUFFER, particles.size() * sizeof( Particle ), nullptr/*particles.data()*/, GL_STATIC_DRAW );
    mParticles[1] = gl::Vbo::create( GL_ARRAY_BUFFER, particles.size() * sizeof( Particle ), nullptr/*particles.data()*/, GL_STATIC_DRAW );
    //mVel = gl::Vbo::create( GL_ARRAY_BUFFER, particles.size() * sizeof(Particle), nullptr, GL_STATIC_DRAW );
    mQuadPositions = gl::Vbo::create( GL_ARRAY_BUFFER, particles.size() * sizeof( vec4 ) * 4, nullptr, GL_STATIC_DRAW );
    
    std::vector<uint32_t> indices( NUM_PARTICLES * 6 );
    // the index buffer is a classic "two-tri quad" array.
    // This may seem odd, given that the compute buffer contains a single
    // vector for each particle.  However, the shader takes care of this
    // by indexing the compute shader buffer with a /4.  The value mod 4
    // is used to compute the offset from the vertex site, and each of the
    // four indices in a given quad references the same center point
    //uint32_t *indices = mIndices->mapT( GL_WRITE_ONLY );
    for( size_t i = 0, j = 0; i < NUM_PARTICLES; ++i ) {
        size_t index = i << 2;
        indices[j++] = index;
        indices[j++] = index + 1;
        indices[j++] = index + 2;
        indices[j++] = index;
        indices[j++] = index + 2;
        indices[j++] = index + 3;
    }
    
    mIndicesVbo = gl::Vbo::create<uint32_t>( GL_ELEMENT_ARRAY_BUFFER, indices, GL_STATIC_DRAW );
    
    for( int i = 0; i < 2; ++i )
    {	// Describe the particle layout for OpenGL.
        mAttributes[i] = gl::Vao::create();
        gl::ScopedVao vao( mAttributes[i] );
        
        // Define attributes as offsets into the bound particle buffer
        gl::ScopedBuffer buffer( mParticles[i] );
        gl::enableVertexAttribArray( 0 );
        gl::enableVertexAttribArray( 1 );
        gl::vertexAttribPointer( 0, 4, GL_FLOAT, GL_FALSE, sizeof( Particle ), reinterpret_cast< const GLvoid *>( offsetof( Particle, pos ) ) );
        gl::vertexAttribPointer( 1, 4, GL_FLOAT, GL_FALSE, sizeof( Particle ), reinterpret_cast<const GLvoid *>( offsetof( Particle, vel ) ) );
    }
    
    /*
    mQuadPosVao = gl::Vao::create();
    gl::ScopedVao vao( mQuadPosVao );
    
    // Define attributes as offsets into the bound particle buffer
    gl::ScopedBuffer buffer( mQuadPositions );
    gl::enableVertexAttribArray( 0 );
    gl::enableVertexAttribArray( 1 );
    gl::enableVertexAttribArray( 2 );
    gl::enableVertexAttribArray( 3 );
    gl::vertexAttribPointer( 0, 4, GL_FLOAT, GL_FALSE, sizeof( vec4 ), reinterpret_cast< const GLvoid *>( sizeof( vec4 ) * 0 ) );
    gl::vertexAttribPointer( 1, 4, GL_FLOAT, GL_FALSE, sizeof( vec4 ), reinterpret_cast< const GLvoid *>( sizeof( vec4 ) * 1 ) );
    gl::vertexAttribPointer( 2, 4, GL_FLOAT, GL_FALSE, sizeof( vec4 ), reinterpret_cast< const GLvoid *>( sizeof( vec4 ) * 2 ) );
    gl::vertexAttribPointer( 3, 4, GL_FLOAT, GL_FALSE, sizeof( vec4 ), reinterpret_cast< const GLvoid *>( sizeof( vec4 ) * 3 ) );
    */
}

void TransformFeedbackApp::update()
{
    updateParticleSystem();
    CI_CHECK_GL();
    
    Particle *particles = static_cast<Particle *>( mParticles[0]->map( GL_WRITE_ONLY ) );
    {
        vec4 p0 = particles[0].pos;
        vec4 p1 = particles[1].pos;
        vec4 p2 = particles[2].pos;
        
        vec4 pl0 = particles[NUM_PARTICLES - 0].pos;
        vec4 pl1 = particles[NUM_PARTICLES - 1].pos;
        vec4 pl2 = particles[NUM_PARTICLES - 2].pos;
        
        vec4 v0 = particles[0].pos;
        vec4 v1 = particles[1].pos;
        vec4 v2 = particles[2].pos;
        
        vec4 vl0 = particles[NUM_PARTICLES - 0].pos;
        vec4 vl1 = particles[NUM_PARTICLES - 1].pos;
        vec4 vl2 = particles[NUM_PARTICLES - 2].pos;
    }
    mParticles[0]->unmap();
    
}

void TransformFeedbackApp::draw()
{
    CI_CHECK_GL();
    gl::clear( ColorA( 0.25f, 0.25f, 0.25f, 1.0f ) );
  }

void TransformFeedbackApp::resetParticleSystem( float size )
{
    Particle *particles = static_cast<Particle *>( mParticles[0]->map( GL_WRITE_ONLY ) );
    {
        //??gl::ScopedBuffer scopePos( mPos );
        for( size_t i = 0; i < NUM_PARTICLES; ++i ) {
            particles[i].pos = vec4( i, i, i, i );
            particles[i].vel = vec4( i, i, i, i );
        }
    }
    mParticles[0]->unmap();
    mSourceIndex = 0;
    mDestinationIndex = 1;
    
    /*
     vec4 *vel = static_cast<vec4 *>( mVel->map( GL_WRITE_ONLY ) );
     {
     //??gl::ScopedBuffer scopePos( mVel );
     for( size_t i = 0; i < NUM_PARTICLES; ++i ) {
     vel[i] = vec4( 0.0f, 0.0f, 0.0f, 1.0f );
     }
     }
     mVel->unmap();
     */
}

void TransformFeedbackApp::updateParticleSystem()
{
    mShaderParams.numParticles = NUM_PARTICLES;
    
    // Todo:  look at uniform buffer objects
    // Invoke the compute shader to integrate the particles
    gl::ScopedGlslProg prog( mUpdateProg );
    gl::ScopedVao source( mAttributes[mSourceIndex] );
    // Bind destination as buffer base.
    gl::bindBufferBase( GL_TRANSFORM_FEEDBACK_BUFFER, 0, mParticles[mDestinationIndex] );
    gl::bindBufferBase( GL_TRANSFORM_FEEDBACK_BUFFER, 1, mQuadPositions );
    
    gl::beginTransformFeedback( GL_POINTS );
    
    // Draw source into destination, performing our vertex transformations.
    gl::drawArrays( GL_POINTS, 0, NUM_PARTICLES ); // NUM_PARTICLES * 6
    
    CI_CHECK_GL();
    
    gl::endTransformFeedback();
    
    // Swap source and destination for next loop
    std::swap( mSourceIndex, mDestinationIndex );
    
    vec4 *quadPositions = static_cast<vec4 *>( mQuadPositions->map( GL_READ_ONLY ) );
    {
        //??gl::ScopedBuffer scopePos( mPos );
        for( size_t i = 0; i < NUM_PARTICLES * 4; ++i ) {
            vec4 *qp = quadPositions[i];
            int j = 0;
        }
    }
    quadPositions->unmap();

    
    
    CI_CHECK_GL();
}

//CINDER_APP_NATIVE( NVidiaComputeParticlesApp, RendererGl )
CINDER_APP( TransformFeedbackApp, RendererGl(),
           [&]( App::Settings *settings ) {
               settings->setWindowSize( 1280, 720 );
           })
#endif
