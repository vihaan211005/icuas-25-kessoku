eval `ssh-agent -s`
export DOCKER_BUILDKIT=1
docker build --ssh default -t icuas_sim_img .
docker start -i icuas_sim_cont
# docker exec -it crazysim_icuas_cont bash
