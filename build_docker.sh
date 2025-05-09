eval `ssh-agent -s`
ssh-add
export DOCKER_BUILDKIT=1
docker build --ssh default -t icuas_sim_img .
