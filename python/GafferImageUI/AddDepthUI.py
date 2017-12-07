##########################################################################
#
#  Copyright (c) 2017, Image Engine Design Inc. All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are
#  met:
#
#      * Redistributions of source code must retain the above
#        copyright notice, this list of conditions and the following
#        disclaimer.
#
#      * Redistributions in binary form must reproduce the above
#        copyright notice, this list of conditions and the following
#        disclaimer in the documentation and/or other materials provided with
#        the distribution.
#
#      * Neither the name of John Haddon nor the names of
#        any other contributors to this software may be used to endorse or
#        promote products derived from this software without specific prior
#        written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
#  IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
#  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
#  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
#  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
#  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
#  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
#  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
#  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
#  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
##########################################################################

import Gaffer
import GafferImage
import IECore

Gaffer.Metadata.registerNode(

	GafferImage.AddDepth,

	"description",
	"""
	Makes sure that an image has the Z and ZBack channels that make it appropriate for use in deep compositing.
	""",

	"layout:activator:depth", lambda node : not node["sourceZChannel"].getValue(),
	"layout:activator:thickness", lambda node : not node["sourceZBackChannel"].getValue(),


	plugs = {

		"depth" : [

			"description",
			"""
			A constant depth value to place the whole image at.
			""",
			"layout:activator", "depth",
		],

		"sourceZChannel" : [

			"description",
			"""
			Use this channel as a Z channel, instead of setting constant depth.
			""",
			"plugValueWidget:type", "GafferImageUI.ChannelPlugValueWidget",
			"channelPlugValueWidget:extraChannels", IECore.StringVectorData( [ "" ] ),
		],

		"thickness" : [

			"description",
			"""
			A constant thickness value for the whole image.  Transparent images will be
			interpreted as fog where the density increases over this range.
			""",
			"layout:activator", "thickness",
		],

		"sourceZBackChannel" : [

			"description",
			"""
			Use this channel as a ZBack channel, instead of setting constant thickness.
			""",
			"plugValueWidget:type", "GafferImageUI.ChannelPlugValueWidget",
			"channelPlugValueWidget:extraChannels", IECore.StringVectorData( [ "" ] ),
			#TODO
			#"channelPlugValueWidget:extraChannelLabels", IECore.StringVectorData( [ "None" ] ),
		],

		


	}

)
