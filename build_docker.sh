eval `ssh-agent -s`
ssh-add
export DOCKER_BUILDKIT=1
docker build -t icuas_competition_img . 
