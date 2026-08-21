"""repo-root pytest가 설치 없이 패키지를 찾도록 경로를 고정한다.

repo 루트(cwd)에서는 outer 디렉토리가 PEP 420 namespace 패키지로 잡혀
``stonefish_albc_bridge.frames`` 해석이 실패한다. 패키지 루트를 sys.path
맨 앞에 넣어 inner regular 패키지가 namespace·설치본 모두에 우선하게 한다.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
