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

using namespace ci;
using namespace ci::app;



inline float sfrand()
{
	//return ( ( randFloat() * 2.0f ) - 1.0f );
	return randPosNegFloat( -1.0f, 1.0f );
}

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
    
    vec4 pos;
    vec4 vel;
};

class NVidiaComputeParticlesApp : public App {
public:
    NVidiaComputeParticlesApp();
    
	virtual void	resize() override;
	virtual void	update() override;
	virtual void	draw() override;
	virtual void	mouseDrag( MouseEvent event ) override;
	virtual void	mouseDown( MouseEvent event ) override;
	
	void renderScene( gl::GlslProgRef effect );
	void setupShaders();
	void setupTextures();
	void setupFbos();
	void setupBuffers();
	void resetParticleSystem( float size );
	void updateParticleSystem();
	void setupNoiseTexture3D();

    enum { NUM_PARTICLES = 1 << 18 };

	gl::GlslProgRef mRenderProg;
	gl::GlslProgRef mUpdateProg;

    // Descriptions of particle data layout.
    gl::VaoRef mParticlesVao[2];
    gl::VaoRef mQuadPositionsVao;
    gl::VboRef mParticles[2];
    gl::VboRef mQuadPositions;
    gl::VboRef mIndicesVbo;
    
    std::uint32_t	mSourceIndex		= 0;
    std::uint32_t	mDestinationIndex	= 1;
    
	gl::Texture3dRef mNoiseTex;
	params::InterfaceGlRef mParams;
    CameraPersp	mCam;
	CameraUi mCamUi;
	ShaderParams mShaderParams;
	int mNoiseSize;
	bool mEnableAttractor;
	bool mAnimate;
	bool mReset;
	float mTime;
	float mPrevElapsedSeconds;
};

NVidiaComputeParticlesApp::NVidiaComputeParticlesApp()
    : mCam( getWindowWidth(), getWindowHeight(), 45.0f, 0.1f, 10.0f ),
      mCamUi( &mCam ),
      mEnableAttractor( false ),
      mAnimate( true ),
      mReset( false ),
      mTime( 0.0f ),
      mPrevElapsedSeconds( 0.0f ),
      mNoiseSize( 16 )
{
    setupNoiseTexture3D();
	setupShaders();
    setupBuffers();
    resetParticleSystem( 0.5f );

    CI_CHECK_GL();
	
	mCam.lookAt( vec3( 0.0f, 0.0f, -3.0f ), vec3( 0 ) );
	
	mParams = params::InterfaceGl::create( "Settings", toPixels( ivec2( 225, 180 ) ) );
	mParams->addSeparator();
	mParams->addParam( "Animate", &mAnimate );
	mParams->addParam( "Enable attractor", &mEnableAttractor );
	mParams->addSeparator();
	mParams->addParam( "Sprite size", &( mShaderParams.spriteSize ) );// Range: 0.0f, 0.04f );
	mParams->addParam( "Noise strength", &( mShaderParams.noiseStrength ) );// Range: 0.0f, 0.01f );
	mParams->addParam( "Noise frequency", &( mShaderParams.noiseFreq ) );// Range: 0.0f, 20.0f );
	mParams->addSeparator();
	mParams->addParam( "Reset", &mReset );
}

void NVidiaComputeParticlesApp::setupShaders()
{
    try {
		mRenderProg = gl::GlslProg::create( gl::GlslProg::Format().vertex( loadAsset( "render.vs.glsl" ) )
			.fragment( loadAsset( "render.fs.glsl" ) ) );
	}
	catch( gl::GlslProgCompileExc e ) {
		ci::app::console() << e.what() << std::endl;
        quit(); //???
	}

   try {
        
        mUpdateProg = gl::GlslProg::create( gl::GlslProg::Format()
                                           .vertex( loadAsset( "particle_update.vs.glsl" ) )
                                           .feedbackFormat( GL_INTERLEAVED_ATTRIBS )
                                           .feedbackVaryings( { "oPosition", "oVelocity", "gl_NextBuffer",
                                                                "oQuadPosition1", "oQuadPosition2",
                                                                "oQuadPosition3", "oQuadPosition4" } )
                                           .attribLocation( "iPosition", 0 )
                                           .attribLocation( "iVelocity", 1 )
                                           );
	}
	catch( gl::GlslProgCompileExc e ) {
		ci::app::console() << e.what() << std::endl;
		quit(); //???
	}

    mUpdateProg->uniform( "invNoiseSize", 1.0f / mNoiseSize );
	mUpdateProg->uniform( "noiseTex3D", 0 );
}

void NVidiaComputeParticlesApp::setupBuffers()
{
    std::vector<Particle> particles( NUM_PARTICLES, Particle() );
    mParticles[0] = gl::Vbo::create( GL_ARRAY_BUFFER, particles.size() * sizeof( Particle ), nullptr, GL_STATIC_DRAW );
    mParticles[1] = gl::Vbo::create( GL_ARRAY_BUFFER, particles.size() * sizeof( Particle ), nullptr, GL_STATIC_DRAW );
    mQuadPositions = gl::Vbo::create( GL_ARRAY_BUFFER, particles.size() * sizeof( vec4 ) * 4, nullptr, GL_STATIC_DRAW );

    // The index buffer is a classic "two-tri quad" array.
	std::vector<uint32_t> indices( NUM_PARTICLES * 6 );
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
        mParticlesVao[i] = gl::Vao::create();
        gl::ScopedVao scopedVao( mParticlesVao[i] );
        
        // Define attributes as offsets into the bound particle buffer
        gl::ScopedBuffer scopedVbo( mParticles[i] );
        gl::enableVertexAttribArray( 0 );
        gl::enableVertexAttribArray( 1 );
        gl::vertexAttribPointer( 0, 4, GL_FLOAT, GL_FALSE, sizeof( Particle ), reinterpret_cast< const GLvoid *>( offsetof( Particle, pos ) ) );
        gl::vertexAttribPointer( 1, 4, GL_FLOAT, GL_FALSE, sizeof( Particle ), reinterpret_cast<const GLvoid *>(offsetof( Particle, vel ) ) );
    }
    
    {
        // TODO:  Fix me.
        mQuadPositionsVao = gl::Vao::create();
        gl::ScopedVao scopedVao( mQuadPositionsVao );
        gl::ScopedBuffer scopedIndices( mIndicesVbo );
        gl::ScopedBuffer scopedPositions( mQuadPositions );
        gl::enableVertexAttribArray( 0 );
        gl::vertexAttribPointer( 0, 4, GL_FLOAT, GL_FALSE, sizeof( vec4 ), reinterpret_cast< const GLvoid *>( 0 ) );
    }
}

void NVidiaComputeParticlesApp::resize()
{
	float vfov = mCam.getFov();
	mCam.setPerspective( vfov, getWindowAspectRatio(), 0.1f, 10.0f ); //! fix hard code
}

void NVidiaComputeParticlesApp::update()
{
	if( mAnimate ) {
		updateParticleSystem();
	}
}

void NVidiaComputeParticlesApp::draw()
{
	gl::clear( ColorA( 0.25f, 0.25f, 0.25f, 1.0f ) );
	gl::clear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

	if( mReset ) {
		mReset = false;
		resetParticleSystem( 0.5f );
	}

	gl::setMatrices( mCam );

	// draw particles
	gl::ScopedGlslProg scopedRenderProg( mRenderProg );
	mRenderProg->uniform( "spriteSize", mShaderParams.spriteSize );

	gl::context()->setDefaultShaderVars();

	gl::enableAdditiveBlending();

	gl::disable( GL_DEPTH_TEST );
	gl::disable( GL_CULL_FACE );
	
	{
        gl::ScopedBuffer scopedIndices( mIndicesVbo );
        gl::ScopedBuffer scopedPositions( mQuadPositions );
        gl::enableVertexAttribArray( 0 );
        gl::vertexAttribPointer( 0, 4, GL_FLOAT, GL_FALSE, sizeof( vec4 ), reinterpret_cast< const GLvoid *>( 0 ) );

        //gl::ScopedVao scopedVao( mQuadPositionsVao );
        gl::drawElements( GL_TRIANGLES, NUM_PARTICLES * 6, GL_UNSIGNED_INT, 0 );
        
        //glDisableVertexAttribArray( 0 );
	}

	CI_CHECK_GL();
	gl::disableAlphaBlending();

	mParams->draw();
}

void NVidiaComputeParticlesApp::mouseDown( MouseEvent event )
{
	mCamUi.mouseDown( event.getPos() );
}

void NVidiaComputeParticlesApp::mouseDrag( MouseEvent event )
{
	mCamUi.mouseDrag( event.getPos(), event.isLeftDown(), event.isMiddleDown(), event.isRightDown() );
}

void NVidiaComputeParticlesApp::resetParticleSystem( float size )
{
	Particle *particles = static_cast<Particle *>( mParticles[0]->map( GL_WRITE_ONLY ) );
    {
        for( size_t i = 0; i < NUM_PARTICLES; ++i ) {
            particles[i].pos = vec4( sfrand() * size, sfrand() * size, sfrand() * size, 1.0f );
            particles[i].vel = vec4( 0.0f, 0.0f, 0.0f, 1.0f );
        }
    }
	mParticles[0]->unmap();
    mSourceIndex = 0;
    mDestinationIndex = 1;
}

void NVidiaComputeParticlesApp::updateParticleSystem()
{
	mShaderParams.numParticles = NUM_PARTICLES;
	if( mEnableAttractor ) {
		// move attractor
		const float speed = 0.2f;
		mShaderParams.attractor.x = math<float>::sin( mTime * speed );
		mShaderParams.attractor.y = math<float>::sin( mTime * speed * 1.3f );
		mShaderParams.attractor.z = math<float>::cos( mTime * speed );
		float elapsedSeconds = static_cast<float>( ci::app::getElapsedSeconds() );
		mTime += elapsedSeconds - mPrevElapsedSeconds;
		mPrevElapsedSeconds = elapsedSeconds;

		mShaderParams.attractor.w = 0.0002f;
	}
	else {
		mShaderParams.attractor.w = 0.0f;
	}
    
    //!!! Todo:  look at uniform buffer objects
    
	// Invoke the compute shader to integrate the particles
	gl::ScopedGlslProg prog( mUpdateProg );
    gl::ScopedState rasterizer( GL_RASTERIZER_DISCARD, true );	// turn off fragment stage
	mUpdateProg->uniform( "attractor", mShaderParams.attractor );
	mUpdateProg->uniform( "numParticles", static_cast<float>( mShaderParams.numParticles ) );
	mUpdateProg->uniform( "damping", mShaderParams.damping );
	mUpdateProg->uniform( "noiseFreq", mShaderParams.noiseFreq );
	mUpdateProg->uniform( "noiseStrength", mShaderParams.noiseStrength );

	gl::ScopedTextureBind scoped3dTex( mNoiseTex );

    gl::ScopedVao scopedSourceVao( mParticlesVao[mSourceIndex] );
    // Bind destination as buffer base.
    gl::bindBufferBase( GL_TRANSFORM_FEEDBACK_BUFFER, 0, mParticles[mDestinationIndex] );
    gl::bindBufferBase( GL_TRANSFORM_FEEDBACK_BUFFER, 1, mQuadPositions );
    gl::beginTransformFeedback( GL_POINTS );
    
    // Draw source into destination, performing our vertex transformations.
    gl::drawArrays( GL_POINTS, 0, NUM_PARTICLES );
    
    gl::endTransformFeedback();
    
    // Swap source and destination for next loop
    std::swap( mSourceIndex, mDestinationIndex );
}

void NVidiaComputeParticlesApp::setupNoiseTexture3D()
{
	gl::Texture3d::Format tex3dFmt;
	tex3dFmt.setWrapR( GL_REPEAT );
	tex3dFmt.setWrapS( GL_REPEAT );
	tex3dFmt.setWrapT( GL_REPEAT );
	tex3dFmt.setMagFilter( GL_LINEAR );
	tex3dFmt.setMinFilter( GL_LINEAR );
	tex3dFmt.setDataType( GL_FLOAT );
	tex3dFmt.setInternalFormat( GL_RGBA8_SNORM );

	const int width = mNoiseSize;
	const int height = mNoiseSize;
	const int depth = mNoiseSize;

	std::vector<float> data( width * height * depth * 4 );
	int i = 0;
	for( int z = 0; z < depth; ++z ) {
		for( int y = 0; y < height; ++y ) {
			for( int x = 0; x < width; ++x ) {
				data[i++] = sfrand();
				data[i++] = sfrand(); 
				data[i++] = sfrand(); 
				data[i++] = sfrand(); 
			}
		}
	}
    
	mNoiseTex = gl::Texture3d::create( mNoiseSize, mNoiseSize, mNoiseSize, tex3dFmt );
	gl::ScopedTextureBind scoped3dTex( mNoiseTex );

    glPixelStorei( GL_UNPACK_ALIGNMENT, 1 );
    glTexImage3D( mNoiseTex->getTarget(),
                  0,
                  mNoiseTex->getInternalFormat(),
                  mNoiseTex->getWidth(),
                  mNoiseTex->getHeight(),
                  mNoiseTex->getDepth(),
                  0.0f,
                  GL_RGBA,
                  tex3dFmt.getDataType(),
                  data.data() );
}

CINDER_APP( NVidiaComputeParticlesApp, RendererGl(),
           [&]( App::Settings *settings ) {
               settings->setWindowSize( 1280, 720 );
           })

