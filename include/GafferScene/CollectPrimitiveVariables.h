//////////////////////////////////////////////////////////////////////////
//
//  Copyright (c) 2018, Image Engine Design Inc. All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions are
//  met:
//
//      * Redistributions of source code must retain the above
//        copyright notice, this list of conditions and the following
//        disclaimer.
//
//      * Redistributions in binary form must reproduce the above
//        copyright notice, this list of conditions and the following
//        disclaimer in the documentation and/or other materials provided with
//        the distribution.
//
//      * Neither the name of John Haddon nor the names of
//        any other contributors to this software may be used to endorse or
//        promote products derived from this software without specific prior
//        written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
//  IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
//  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
//  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
//  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
//  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
//  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
//  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
//  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
//  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//////////////////////////////////////////////////////////////////////////

#ifndef GAFFERSCENE_COLLECTPRIMITIVEVARIABLES_H
#define GAFFERSCENE_COLLECTPRIMITIVEVARIABLES_H

#include "GafferScene/SceneElementProcessor.h"

namespace Gaffer
{

IE_CORE_FORWARDDECLARE( StringPlug )

} // namespace Gaffer

namespace GafferScene
{

class GAFFERSCENE_API CollectPrimitiveVariables : public SceneElementProcessor
{

	public :

		CollectPrimitiveVariables( const std::string &name=defaultName<CollectPrimitiveVariables>() );
		~CollectPrimitiveVariables() override;

		GAFFER_GRAPHCOMPONENT_DECLARE_TYPE( GafferScene::CollectPrimitiveVariables, CollectPrimitiveVariablesTypeId, SceneElementProcessor );

		Gaffer::StringPlug *primitiveVariablesPlug();
		const Gaffer::StringPlug *primitiveVariablesPlug() const;

		Gaffer::StringVectorDataPlug *suffixesPlug();
		const Gaffer::StringVectorDataPlug *suffixesPlug() const;

		Gaffer::StringPlug *suffixContextVariablePlug();
		const Gaffer::StringPlug *suffixContextVariablePlug() const;

		Gaffer::BoolPlug *requireVariationPlug();
		const Gaffer::BoolPlug *requireVariationPlug() const;

		void affects( const Gaffer::Plug *input, AffectedPlugsContainer &outputs ) const override;

	protected :

		bool processesObject() const override;
		void hashProcessedObject( const ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h ) const override;
		IECore::ConstObjectPtr computeProcessedObject( const ScenePath &path, const Gaffer::Context *context, IECore::ConstObjectPtr inputObject ) const override;

	private :

		static size_t g_firstPlugIndex;

};

IE_CORE_DECLAREPTR( CollectPrimitiveVariables )

} // namespace GafferScene

#endif // GAFFERSCENE_COLLECTPRIMITIVEVARIABLES_H
