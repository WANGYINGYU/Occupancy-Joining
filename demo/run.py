#!/usr/bin/env python3
"""Download demo inputs and run Occupancy Joining with Docker or native ROS 2."""
import argparse,hashlib,json,os,signal,shutil,subprocess,sys,tarfile,time,urllib.request,uuid
from pathlib import Path,PurePosixPath
ROOT=Path(__file__).resolve().parents[1]

def digest(path):
    with path.open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()

def verify(path,size,checksum):
    if path.stat().st_size!=size or digest(path)!=checksum:raise ValueError(f'Input does not match this demo release: {path}')

def download(url,path,size,checksum):
    print(f'Downloading {path.name} ({size/1e6:.1f} MB)...',flush=True)
    with urllib.request.urlopen(url,timeout=60) as response,path.open('xb') as output:
        received=0
        while chunk:=response.read(1024*1024):
            received+=len(chunk)
            if received>size:raise ValueError('Download exceeds the published input size')
            output.write(chunk)
    verify(path,size,checksum)
    return path

def unpack(archive,destination,root_name,expected_bytes):
    destination.mkdir(parents=True,exist_ok=False)
    with tarfile.open(archive,'r:gz') as tar:
        members=tar.getmembers();names=set();total=0
        for member in members:
            name=PurePosixPath(member.name)
            if name.is_absolute() or '..' in name.parts or '\\' in member.name or not name.parts or name.parts[0]!=root_name or member.name in names or not (member.isfile() or member.isdir()):raise ValueError(f'Unsafe demo archive entry: {member.name}')
            names.add(member.name);total+=member.size
        if total!=expected_bytes or len(members)>64:raise ValueError('Unexpected demo archive contents')
        if shutil.disk_usage(destination).free<total+100_000_000:raise ValueError('Insufficient extraction space')
        for member in members:
            target=destination/member.name
            if member.isdir():target.mkdir(parents=True,exist_ok=True)
            else:
                target.parent.mkdir(parents=True,exist_ok=True)
                with tar.extractfile(member) as source,target.open('xb') as output:shutil.copyfileobj(source,output)

def bind(path,readonly=False,target=None):
    value=f'type=bind,source={path},target={target or path}'
    if ',' in str(path) or ',' in str(target or path):raise ValueError('Docker mount paths cannot contain commas')
    return ['--mount',value+(',readonly' if readonly else '')]

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--demo',choices=['room','intel','b2'],default='room');p.add_argument('--work',type=Path,required=True,help='New directory for downloaded inputs, results and logs')
    p.add_argument('--native',action='store_true',help='Use the sourced native ROS installation instead of Docker')
    p.add_argument('--archive',type=Path,help='Previously downloaded, unmodified demo input archive')
    p.add_argument('--bag',type=Path,help='Original b2 bag; otherwise downloaded from the original provider')
    p.add_argument('--image',default=os.environ.get('OCCUPANCY_JOINING_DOCKER_IMAGE','occupancy-joining:jazzy'))
    p.add_argument('--mount-input',type=Path,action='append',default=[],help='Additional read-only directory to mount at the same absolute path (Docker)')
    p.add_argument('--timeout',type=int,default=3600)
    a=p.parse_args();meta=json.loads((ROOT/'demo'/f'{a.demo}.json').read_text());work=a.work.expanduser().resolve()
    if work.exists():p.error('--work must be a new directory')
    if work.is_relative_to(ROOT):p.error('Keep demo inputs/results outside the source checkout')
    if a.timeout<=0:p.error('Use a positive timeout')
    if a.bag and a.demo!='b2':p.error('--bag is only used by the b2 demo')
    work.mkdir(parents=True);runtime=work/'runtime';runtime.mkdir();env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1')
    env.pop('OCCUPANCY_JOINING_RUN_DIR',None)
    for key,sub in [('TMPDIR','tmp'),('TMP','tmp'),('TEMP','tmp'),('ROS_HOME','ros'),('ROS_LOG_DIR','ros_logs'),('XDG_CACHE_HOME','cache'),('MPLCONFIGDIR','matplotlib'),('PYTHONPYCACHEPREFIX','pycache')]:
        folder=runtime/sub;folder.mkdir(exist_ok=True);env[key]=str(folder)
    for key in ['OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS']:env[key]='1'
    container=None;process=None;started=time.monotonic()
    try:
        archive=a.archive.expanduser().resolve(strict=True) if a.archive else download(meta['download_url'],work/meta['archive_name'],meta['archive_bytes'],meta['archive_sha256'])
        verify(archive,meta['archive_bytes'],meta['archive_sha256']);unpack(archive,work/'data',a.demo,meta['archive_unpacked_bytes'])
        bag=None
        if a.demo=='b2':
            raw=meta['raw'];bag=a.bag.expanduser().resolve(strict=True) if a.bag else download(raw['url'],work/raw['name'],raw['bytes'],raw['sha256'])
            verify(bag,raw['bytes'],raw['sha256'])
        args=['--demo',a.demo,'--data',str(work/'data'/a.demo),'--output',str(work/'results')]
        if bag:args+=['--bag',str(bag)]
        if a.native:
            backend=Path(env.get('OCCUPANCY_JOINING_BACKEND',ROOT/'install/occupancy_joining/lib/occupancy_joining/joining_backend')).expanduser().resolve()
            if not backend.is_file():raise ValueError('Build this checkout and source install/setup.bash first')
            env['OCCUPANCY_JOINING_BACKEND']=str(backend);command=[sys.executable,str(ROOT/'demo/worker.py'),*args]
        else:
            if not shutil.which('docker'):raise ValueError('Install Docker Engine first')
            build_env=dict(env,OCCUPANCY_JOINING_WORKSPACE=str(work/'build_workspace'))
            subprocess.run(['bash',str(ROOT/'tools/docker.sh'),'--image',a.image,'build'],env=build_env,check=True)
            container='occupancy-joining-demo-'+uuid.uuid4().hex[:12]
            command=['docker','run','--name',container,'--rm','--init','--read-only','--network','none','--log-driver','none','--cap-drop','ALL','--security-opt','no-new-privileges','--user',f'{os.getuid()}:{os.getgid()}','--workdir',str(work)]
            for extra in a.mount_input:
                extra=extra.expanduser().resolve(strict=True)
                if not extra.is_dir() or extra==Path(extra.anchor):raise ValueError('Additional input mount must be a directory other than the filesystem root')
                command+=bind(extra,readonly=True)
            command+=bind(work)+bind(work/'data',readonly=True)
            if bag and not bag.is_relative_to(work):command+=bind(bag,readonly=True)
            for sub,target in [('container_tmp','/tmp'),('container_var_tmp','/var/tmp'),('container_shm','/dev/shm')]:
                folder=runtime/sub;folder.mkdir();command+=bind(folder,target=target)
            for key in ['TMPDIR','TMP','TEMP','ROS_HOME','ROS_LOG_DIR','XDG_CACHE_HOME','MPLCONFIGDIR','PYTHONPYCACHEPREFIX','PYTHONDONTWRITEBYTECODE','OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS']:command+=['--env',f'{key}={env[key]}']
            image_id=subprocess.check_output(['docker','image','inspect',a.image,'--format','{{.Id}}'],text=True).strip();(work/'image.json').write_text(json.dumps(dict(image=a.image,id=image_id),indent=2))
            command+=[a.image,'exec','python3','/opt/occupancy-joining/demo/worker.py',*args]
        (work/'command.json').write_text(json.dumps(command,indent=2));print('Running demo...',flush=True)
        with (work/'run.log').open('w') as log:
            process=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            code=process.wait(timeout=a.timeout)
        if code:raise RuntimeError(f'Demo exited {code}; see {work/"run.log"}')
        (work/'status.json').write_text(json.dumps(dict(status='completed',demo=a.demo,frames=meta['frames'],seconds=time.monotonic()-started),indent=2));print(f'Completed: {work/"results/index.html"}',flush=True);print(f'RViz result directory: {work/"results"}',flush=True)
    except BaseException as exc:
        (work/'status.json').write_text(json.dumps(dict(status='failed',demo=a.demo,error=str(exc),seconds=time.monotonic()-started),indent=2));raise
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=20)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        if container and subprocess.run(['docker','inspect',container],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode==0:subprocess.run(['docker','stop','--time','20',container],stdout=subprocess.DEVNULL,check=False)

if __name__=='__main__':
    try:main()
    except (OSError,ValueError,RuntimeError,subprocess.CalledProcessError,subprocess.TimeoutExpired) as exc:sys.exit(f'Demo failed: {exc}')
