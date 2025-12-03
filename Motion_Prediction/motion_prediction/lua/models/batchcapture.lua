require("config")

package.projectPath='../Samples/QP_controller/'
package.path=package.path..";../Samples/QP_controller/lua/?.lua" --;"..package.path
package.path=package.path..";../Samples/classification/lua/?.lua" --;"..package.path
package.path=package.path..";../Samples/ysscripts/samples/?.lua" --;"..package.path
package.path=package.path..";../Samples/ysscripts/lib/?.lua" --;"..package.path

require('driverDynamicMuscle')

ctor_old=ctor

function ctor()
	ctor_old()

	local captureUntil=1200
	--local captureUntil=300
	local tryFunc=function(captureUntil)
		mEventReceiver:onFrameChanged(win, captureUntil)
	end

	local output={pcall(tryFunc, captureUntil)}
	if output[1]==false then
		print(output[2])
	end
	mEventReceiver:attachCamera()

	if true then -- capture jpeg sequence
		RE.renderer():setScreenshotPrefix('../dump/'..prefix)
		RE.renderer():screenshot(true)
		for i=0,g_prev_iframe-1 , 4 do
			mEventReceiver:onFrameChanged(win,i)
			RE.renderOneFrame(false)
		end
	end
	RE.renderer():screenshot(false)
	this('exit!',0)
end
