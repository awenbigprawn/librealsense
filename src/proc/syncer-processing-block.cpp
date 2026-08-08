// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 RealSense, Inc. All Rights Reserved.

#include <functional>
#include "source.h"
#include "sync.h"
#include "proc/synthetic-stream.h"
#include "proc/syncer-processing-block.h"
#include <src/core/frame-processor-callback.h>

#ifdef RS_CAMERA_V4L2_DIAGNOSTICS
#include "../linux/v4l2-diagnostic-trace.h"
#define RS_SYNCER_DIAGNOSTIC_RECORD(...) platform::v4l2_diagnostic::record(__VA_ARGS__)
#else
#define RS_SYNCER_DIAGNOSTIC_RECORD(...) ((void)0)
#endif


namespace librealsense
{
    syncer_process_unit::syncer_process_unit(std::initializer_list< bool_option::ptr > enable_opts, bool log)
        : processing_block("syncer"), _matcher((new composite_identity_matcher({})))
        , _enable_opts(enable_opts.begin(), enable_opts.end())
    {
        _matcher->set_callback( []( frame_holder f, syncronization_environment const & env ) {
            if( env.log )
            {
                LOG_DEBUG( "<-- queueing " << f );
            }

            // We get here from within a dispatch() call, already protected by a mutex -- so only
            // one thread can enqueue!
            env.matches.enqueue( std::move( f ) );
        } );

        // This callback gets called by the previous processing block when it is done with a frame. We
        // call the matchers with the frame and eventually call the next callback in the list using frame_ready().
        // This callback can get called from multiple threads, one thread per stream -- but always in the correct
        // frame order per stream.
        auto f = [&, log](frame_holder && frame, synthetic_source_interface* source)
        {
            auto const diagnostic_frame_number = frame && frame.frame
                ? static_cast< uint32_t >( frame.frame->get_frame_number() )
                : 0;
            // if the syncer is disabled passthrough the frame
            bool enabled = false;
            size_t n_opts = 0;
            for (auto& wopt : _enable_opts)
            {
                auto opt = wopt.lock();
                if (opt)
                {
                    ++n_opts;
                    if (opt->is_true())
                    {
                        enabled = true;
                        break;
                    }
                }
            }
            if (n_opts && !enabled)
            {
                get_source().frame_ready(std::move(frame));
                return;
            }
            LOG_DEBUG( "--> syncing " << frame );
            RS_SYNCER_DIAGNOSTIC_RECORD(
                platform::v4l2_diagnostic::stage::syncer_match_begin,
                -1,
                0,
                diagnostic_frame_number );
            {
                std::lock_guard<std::mutex> lock(_mutex);
                if( ! _matcher->get_active() )
                {
                    LOG_DEBUG( "matcher was stopped: NOT DISPATCHING FRAME!" );
                    return;
                }
                _matcher->dispatch(std::move(frame), { source, _matches, log });
            }
            RS_SYNCER_DIAGNOSTIC_RECORD(
                platform::v4l2_diagnostic::stage::syncer_match_end,
                -1,
                0,
                diagnostic_frame_number );

            frame_holder f;
            {
                // Another thread has the lock, meaning will get into the following loop and dequeue all
                // the frames. So there's nothing for us to do...
                std::unique_lock< std::mutex > lock(_callback_mutex, std::try_to_lock);
                if (!lock.owns_lock())
                    return;

                while (_matches.try_dequeue(&f))
                {
                    LOG_DEBUG( "--> frame ready: " << *f.frame );
                    auto const emitted_frame_number = f && f.frame
                        ? static_cast< uint32_t >( f.frame->get_frame_number() )
                        : 0;
                    RS_SYNCER_DIAGNOSTIC_RECORD(
                        platform::v4l2_diagnostic::stage::syncer_emit_begin,
                        -1,
                        0,
                        emitted_frame_number );
                    get_source().frame_ready(std::move(f));
                    RS_SYNCER_DIAGNOSTIC_RECORD(
                        platform::v4l2_diagnostic::stage::syncer_emit_end,
                        -1,
                        0,
                        emitted_frame_number );
                }
            }

        };

        set_processing_callback( make_frame_processor_callback( std::move( f ) ) );
    }

    // Stopping the syncer means no more frames will be enqueued, and any existing frames
    // pending dispatch will be lost!
    void syncer_process_unit::stop()
    {
        _matcher->stop();
    }
}
