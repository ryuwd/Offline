yum -y install epel-release
yum -y install make base-devel glibc-devel freetype-devel xxhash-devel xxhash-libs libcurl libcurl-devel

source /cvmfs/fermilab.opensciencegrid.org/products/common/etc/setups
setup mu2e
source setup.sh

scons -k --max-drift=1 --implicit-deps-unchanged -j 24 2>&1 | tee "${WORKSPACE}/scons.log"

declare -a FCLFILES=(
    "Validation/fcl/ceSimReco.fcl" 
    "Mu2eG4/fcl/g4test_03MT.fcl" 
    "Mu2eG4/fcl/transportOnly.fcl" 
    "JobConfig/beam/PS.fcl" 
    "Mu2eG4/fcl/g4study.fcl" 
    "Validation/fcl/cosmicSimReco.fcl")

mu2e -c Validation/fcl/ceSimReco.fcl -n 1
mu2e -c Mu2eG4/fcl/g4test_03MT.fcl -n 10
mu2e -c Mu2eG4/fcl/transportOnly.fcl -n 1 
mu2e -c JobConfig/beam/PS.fcl -n 1
mu2e -c Mu2eG4/fcl/g4study.fcl -n 1
mu2e -c Validation/fcl/cosmicSimReco.fcl -n 1


# if we get this far, the build passes
