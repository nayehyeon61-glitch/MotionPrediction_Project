require("config")

package.projectPath='../Samples/QP_controller/'
package.path=package.path..";../Samples/QP_controller/lua/?.lua" --;"..package.path
package.path=package.path..";../Samples/classification/lua/?.lua" --;"..package.path
package.path=package.path..";../Samples/ysscripts/samples/?.lua" --;"..package.path
package.path=package.path..";../Samples/ysscripts/lib/?.lua" --;"..package.path

require('showMuscles')

ctor_old=ctor

function ctor()
	ctor_old()

	local captureUntil=1200
	--local captureUntil=120
	mEventReceiver:attachCamera()

	if true then -- capture jpeg sequence
		RE.renderer():setScreenshotPrefix('../dump/'..prefix)
		RE.renderer():screenshot(true)
		--for i=0,g_prev_iframe-1 , 4 do
		for i=0,mMotionDOFcontainer.mot:rows()-1 , 4 do
			if i < captureUntil then
				mEventReceiver:onFrameChanged(win,i)
				mOsim.mSkin:setPoseDOF(mMotionDOFcontainer.mot:row(i))
				RE.renderOneFrame(true)
			end
		end
	end
	RE.renderer():screenshot(false)
	this('exit!',0)
end
