
function init_debug_globals()
	exportDebugInfo=false -- mrd_info
	-- change export_freq in wrlviewer.lua too.
	mrd_info={ filename="debug_plot.mrd", min_frame=0, export_freq=50, outputMotion=true, outputContactForce=true }
	mrd_channel_strings={
		-- "rclavicle_theta", 
		-- "rclavicle_theta_d", 
		-- "rclavicle_dtheta", 
		-- "rclavicle_dtheta_d",
		-- "rclavicle_torque",
		-- "delta", 
		"COMx", 
		"COMy", 
		"COMz",
	}
end
