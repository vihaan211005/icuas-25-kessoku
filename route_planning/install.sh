sudo apt update
sudo apt install -y build-essential cmake lsb-release
wget https://github.com/google/or-tools/releases/download/v9.11/or-tools_amd64_ubuntu-24.04_cpp_v9.11.4210.tar.gz
tar -xvzf or-tools_amd64_ubuntu-24.04_cpp_v9.11.4210.tar.gz
mv or-tools_x86_64_Ubuntu-24.04_cpp_v9.11.4210  or_tools
rm -rf or-tools_amd64_ubuntu-24.04_cpp_v9.11.4210.tar.gz
