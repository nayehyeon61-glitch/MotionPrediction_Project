##Sensor viwer##
record_viewer:
	python3 ../python/showBinFile.py
record_transfer:
	python3 ../python/realtime_posetransfer.py
record_transfer2:
	python3 ../python/realtime_posetransfer2.py
record_compare:
	python3 ../python/bin_viewer.py
record_transfer2:
	python3 ../python/realtime_posetransfer2.py
## motion prediction ##
train_hand_model:
	python3 train_hand_model.py gym_mp/hand_env.lua
train_motionprediction_ae:
	python3 train_motionprediction.py gym_mp/MotionPrediction_env_ae.lua
motionprediction_ae:
	python3 play_mp.py gym_mp/MotionPrediction_env_ae.lua
train_motionprediction_vae:
	python3 train_motionprediction_vae.py gym_mp/vae_test_10.lua
motionprediction_vae:
	python3 play_mp.py gym_mp/MotionPrediction_env_vae.lua

## vqvae prediction
test_mp_vqvae:
	python3 train_mp_vavae.py False

train_mp_vqvae:
	python3 train_mp_vavae.py True

## diffusion model based prediction
train_mp_dif16:
	python3 train_motionprediction_dif16.py gym_mp/dif_test_10.lua
motionprediction_dif16:
	python3 train_motionprediction_dif16.py False

## vqvae new version prediction
motionprediction_vqvae:
	python3 train_mp_vqvae_Categorical.py False

train_mp_vqvae_categorical:
	python3 train_mp_vqvae_Categorical.py True
