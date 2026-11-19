"""Restore the exact vocabulary from the provided Release ZIP. Standard library only."""
from pathlib import Path
import argparse,hashlib,json,os,sys,tempfile,zipfile
ROOT=Path(__file__).resolve().parent
def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
 return h.hexdigest()
def main():
 parser=argparse.ArgumentParser(description=__doc__)
 parser.add_argument('archive',type=Path)
 args=parser.parse_args()
 a=json.loads((ROOT/'RELEASE_ASSETS.json').read_text())['vocabulary']
 tmp=None
 try:
  if not args.archive.is_file():raise ValueError('Vocabulary ZIP not found')
  if args.archive.stat().st_size!=a['archive_bytes'] or sha(args.archive)!=a['archive_sha256']:raise ValueError('Vocabulary ZIP hash mismatch')
  dest=ROOT/a['destination']
  if dest.exists():
   if dest.is_file() and dest.stat().st_size==a['bytes'] and sha(dest)==a['sha256']:
    print(json.dumps({'status':'completed','action':'already_present'}));return 0
   raise ValueError('Refusing to overwrite a different existing vocabulary')
  with zipfile.ZipFile(args.archive) as z:
   if z.namelist()!=[a['member']] or z.getinfo(a['member']).file_size!=a['bytes']:raise ValueError('Unexpected archive contents')
   dest.parent.mkdir(parents=True,exist_ok=True)
   fd,name=tempfile.mkstemp(prefix='.orbvoc-',dir=dest.parent);tmp=Path(name)
   with os.fdopen(fd,'wb') as out,z.open(a['member']) as inp:
    for block in iter(lambda:inp.read(1024*1024),b''):out.write(block)
  if tmp.stat().st_size!=a['bytes'] or sha(tmp)!=a['sha256']:raise ValueError('Extracted vocabulary hash mismatch')
  # Exclusive creation prevents overwriting a file created during extraction.
  os.link(tmp,dest)
  print(json.dumps({'status':'completed','destination':str(dest)}));return 0
 except KeyboardInterrupt:
  print(json.dumps({'status':'cancelled'}),file=sys.stderr);return 130
 except Exception as e:
  print(json.dumps({'status':'failed','error':str(e)}),file=sys.stderr);return 1
 finally:
  if tmp is not None:tmp.unlink(missing_ok=True)
if __name__=='__main__':sys.exit(main())
