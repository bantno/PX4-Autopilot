############################################################################
#
#   Copyright (c) 2025 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################


# Animated Gerstner-wave water surface for camera views (visual only; the AHAB water physics stay flat).
# Wavefield / WaveVisual / PublisherPlugin are the gz-sim "surface vehicles" example plugins (Apache-2.0, see the
# file headers), as shipped in the lyoko_gz_maritime package.
project(MaritimeWaveVisual)

find_package(gz-rendering NAMES gz-rendering8 gz-rendering9 REQUIRED)
find_package(gz-common NAMES gz-common5 gz-common6 COMPONENTS graphics REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} SHARED
    Wavefield.cc
    WaveVisual.cc
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC Eigen3::Eigen
    PUBLIC gz-rendering${gz-rendering_VERSION_MAJOR}::gz-rendering${gz-rendering_VERSION_MAJOR}
    PUBLIC gz-common${gz-common_VERSION_MAJOR}::gz-common${gz-common_VERSION_MAJOR}
    PUBLIC ${GZ_PLUGIN_TARGET}
    PUBLIC ${GZ_SIM_TARGET}
    PUBLIC ${GZ_TRANSPORT_TARGET}
)

# Publishes the wavefield parameters (gain, period, direction) the WaveVisual subscribes to
add_library(MaritimeParamPublisher SHARED
    PublisherPlugin.cc
)

target_link_libraries(MaritimeParamPublisher
    PUBLIC ${GZ_PLUGIN_TARGET}
    PUBLIC ${GZ_SIM_TARGET}
    PUBLIC ${GZ_TRANSPORT_TARGET}
)

# vendored upstream code: keep it unmodified, so PX4's strict warning set is not an error here
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-error)
target_compile_options(MaritimeParamPublisher PRIVATE -Wno-error)
