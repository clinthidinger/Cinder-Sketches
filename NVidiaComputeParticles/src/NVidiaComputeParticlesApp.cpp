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

#if 1
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


//#define LIGHT_RADIUS	1.5f // Must be at least 1

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
    
    vec4	pos;
    vec4	vel;
};

class NVidiaComputeParticlesApp : public App {
public:
    NVidiaComputeParticlesApp();
    
	//void	setup() override;
	virtual void	resize() override;
	virtual void	update() override;
	virtual void	draw() override;
	virtual void	mouseDrag( MouseEvent event ) override;
	virtual void	mouseDown( MouseEvent event ) override;
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
    gl::VaoRef      mQuadPositionsVao;
    // Buffers holding raw particle data on GPU.
    //gl::VboRef		mParticleBuffer[2];
    
    std::uint32_t	mSourceIndex		= 0;
    std::uint32_t	mDestinationIndex	= 1;
    gl::VboRef mParticles[2];
    gl::VboRef mQuadPositions;
    //gl::VboRef mVel;
	gl::VboRef mIndicesVbo;

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
    //ensureAssetDirsPrepared();//addAssetDirectory( "assets" );
    setupNoiseTexture3D();
    CI_CHECK_GL();
	setupShaders();
    CI_CHECK_GL();
	setupBuffers();
    CI_CHECK_GL();

	resetParticleSystem( 0.5f );

	//CameraPersp cam( mMayaCam.getCamera() );
	mCam.lookAt( vec3( 0.0f, 0.0f, -3.0f ), vec3( 0 ) );
	//mCam.setCenterOfInterestPoint( vec3( 0 ) );
	//mCam.setNearClip( 0.10f );
	//mCam.setFarClip( 10.0f );
	//mCamUi..setCurrentCam( cam );

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
    CI_CHECK_GL();
	try {
		mRenderProg = gl::GlslProg::create( gl::GlslProg::Format().vertex( loadAsset( "render.vs.glsl" ) )
			.fragment( loadAsset( "render.fs.glsl" ) ) );
	}
	catch( gl::GlslProgCompileExc e ) {
		ci::app::console() << e.what() << std::endl;
        quit(); //???
	}

    CI_CHECK_GL();
	try {
        
        mUpdateProg = gl::GlslProg::create( gl::GlslProg::Format()
                                           .vertex( loadAsset( "particle_update.vs.glsl" ) )
                                           .feedbackFormat( GL_INTERLEAVED_ATTRIBS )
                                           .feedbackVaryings( { "oPosition", "oVelocity", "gl_NextBuffer",
                                                                "oQuadPosition1", "oQuadPosition2", "oQuadPosition3", "oQuadPosition4" } )
                                           .attribLocation( "iPosition", 0 )
                                           .attribLocation( "iVelocity", 1 )
                                           );
	}
	catch( gl::GlslProgCompileExc e ) {
		ci::app::console() << e.what() << std::endl;
		quit(); //???
	}

     CI_CHECK_GL();
	mUpdateProg->uniform( "invNoiseSize", 1.0f / mNoiseSize );
	mUpdateProg->uniform( "noiseTex3D", 0 );
}

void NVidiaComputeParticlesApp::setupBuffers()
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
        gl::vertexAttribPointer( 1, 4, GL_FLOAT, GL_FALSE, sizeof( Particle ), reinterpret_cast<const GLvoid *>(offsetof( Particle, vel ) ) );
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
	CI_CHECK_GL();
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
        //gl::ScopedBufferBase scopedPosBuffer( mPos, 1 );
        //gl::ScopedBuffer scopedIndicex( mIndicesVbo );
        gl::ScopedBuffer scopedIndicex( mIndicesVbo );
        //glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, mIndicesVbo );
        //glBindBuffer( GL_ARRAY_BUFFER, mIndicesVbo );
        //gl::ScopedBuffer scopedPositions( mQuadPositions->getTarget(), 1 );
        gl::ScopedBuffer scopedPositions( mQuadPositions );

        glEnableVertexAttribArray( 0 );
        glVertexAttribPointer( 0, 4, GL_FLOAT, GL_FALSE, sizeof( vec4 ), reinterpret_cast< const GLvoid *>( 0 ) );

        
        gl::drawElements( GL_TRIANGLES, NUM_PARTICLES * 6, GL_UNSIGNED_INT, 0 );
        
        glDisableVertexAttribArray( 0 );
        //gl::vertexAttribPointer( 0, 4, GL_FLOAT, GL_FALSE, sizeof( vec4 ), reinterpret_cast< const GLvoid *>( 0 ) );
        
		//gl::drawElements( GL_TRIANGLES, NUM_PARTICLES * 4, GL_FLOAT, 0 );
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
        //??gl::ScopedBuffer scopePos( mPos );
        for( size_t i = 0; i < NUM_PARTICLES; ++i ) {
            particles[i].pos = vec4( sfrand() * size, sfrand() * size, sfrand() * size, 1.0f );
            particles[i].vel = vec4( 0.0f, 0.0f, 0.0f, 1.0f );
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
    
    // Todo:  look at uniform buffer objects
    
	// Invoke the compute shader to integrate the particles
	gl::ScopedGlslProg prog( mUpdateProg );
    gl::ScopedState rasterizer( GL_RASTERIZER_DISCARD, true );	// turn off fragment stage
	mUpdateProg->uniform( "attractor", mShaderParams.attractor );
	//mUpdateProg->uniform( "numParticles", static_cast<int>( mShaderParams.numParticles ) ); //fix this???
	mUpdateProg->uniform( "numParticles", static_cast<float>( mShaderParams.numParticles ) ); //fix this???
	//mUpdateProg->uniform( "spriteSize", mShaderParams.spriteSize );
	mUpdateProg->uniform( "damping", mShaderParams.damping );
	mUpdateProg->uniform( "noiseFreq", mShaderParams.noiseFreq );
	mUpdateProg->uniform( "noiseStrength", mShaderParams.noiseStrength );

	gl::ScopedTextureBind scoped3dTex( mNoiseTex );

     CI_CHECK_GL();
	//ScopedBufferBase scopedPosBuffer( mPos, 1 );
	//ScopedBufferBase scopedVelBuffer( mVel, 2 );

	//glDispatchCompute( NUM_PARTICLES / WORK_GROUP_SIZE, 1, 1 );
	// We need to block here on compute completion to ensure that the
	// computation is done before we render
	//glMemoryBarrier( GL_SHADER_STORAGE_BARRIER_BIT );
    
    //gl::drawElements( GL_TRIANGLES, NUM_PARTICLES * 6, GL_UNSIGNED_INT, 0 );
    // Bind the source data (Attributes refer to specific buffers).
    gl::ScopedVao source( mAttributes[mSourceIndex] );
    // Bind destination as buffer base.
    gl::bindBufferBase( GL_TRANSFORM_FEEDBACK_BUFFER, 0, mParticles[mDestinationIndex] );
    gl::bindBufferBase( GL_TRANSFORM_FEEDBACK_BUFFER, 1, mQuadPositions );
    gl::beginTransformFeedback( GL_POINTS );
    
    CI_CHECK_GL();
    
    // Draw source into destination, performing our vertex transformations.
    gl::drawArrays( GL_POINTS, 0, NUM_PARTICLES ); // NUM_PARTICLES * 6
    
    CI_CHECK_GL();
    gl::endTransformFeedback();
    
    // Swap source and destination for next loop
    std::swap( mSourceIndex, mDestinationIndex );
    
    /*
    vec4 *quadPositions = static_cast<vec4 *>( mQuadPositions->map( GL_READ_ONLY ) );
    {
        for( size_t i = 0; i < ( NUM_PARTICLES ); ++i ) {
            vec4 qp = quadPositions[i * 4];
            ci::app::console() << i << ") " << qp << std::endl;
        }
    }
    mQuadPositions->unmap();
    //*/
    
	CI_CHECK_GL();
}

void NVidiaComputeParticlesApp::setupNoiseTexture3D()
{
	gl::Texture3d::Format tex3dFmt;
	tex3dFmt.setWrapR( GL_REPEAT );
	tex3dFmt.setWrapS( GL_REPEAT );
	tex3dFmt.setWrapT( GL_REPEAT );
	tex3dFmt.setMagFilter( GL_LINEAR );
	tex3dFmt.setMinFilter( GL_LINEAR );
	//tex3dFmt.setInternalFormat( GL_RGBA );
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
    CI_CHECK_GL();
	mNoiseTex = gl::Texture3d::create( mNoiseSize, mNoiseSize, mNoiseSize, tex3dFmt );
	gl::ScopedTextureBind scoped3dTex( mNoiseTex );

    CI_CHECK_GL();
	glPixelStorei( GL_UNPACK_ALIGNMENT, 1 );
    CI_CHECK_GL();
	glTexImage3D( mNoiseTex->getTarget(),
                  0,
                  mNoiseTex->getInternalFormat(),
                  mNoiseTex->getWidth(),
                  mNoiseTex->getHeight(),
                  mNoiseTex->getDepth(),
                  0.0f,
                  GL_RGBA,//GL_RGBA8_SNORM,//tex3dFmt.getInternalFormat(),
                  tex3dFmt.getDataType(), //!!! Possibly copy and paste bug!!!!
                  data.data() );//&( data[0] ) );
    CI_CHECK_GL();
}

//CINDER_APP_NATIVE( NVidiaComputeParticlesApp, RendererGl )
CINDER_APP( NVidiaComputeParticlesApp, RendererGl(),
           [&]( App::Settings *settings ) {
               settings->setWindowSize( 1280, 720 );
           })
#endif
