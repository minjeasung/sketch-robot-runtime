"""Apply only generated SRDF collision exclusions to the imported physics bodies."""
from pathlib import Path
import hashlib
import xml.etree.ElementTree as ET


def _links(path):
    nodes=ET.parse(path).getroot().findall('link')
    result={node.get('name'):node for node in nodes}
    if len(result)!=len(nodes) or any(not name for name in result):
        raise ValueError('Robot link names must be unique and nonempty')
    return result


def collision_filter_plan(bundle):
    """Pure description mapping; never infer exclusions from proximity or ancestry."""
    full=_links(bundle['urdf']);physical=_links(bundle['physics_urdf'])
    if not set(physical)<=set(full):
        raise ValueError('Physics description contains links outside generated URDF')
    mapping={name:name for name in physical}
    reduced={}
    for boundary in bundle.get('ft_boundaries',{}).values():
        body=boundary['child'];descendants=boundary['downstream_links']
        if body not in physical or body not in descendants or not set(descendants)<=set(full):
            raise ValueError('Invalid reduced roller body mapping')
        for name in descendants:
            if name in reduced and reduced[name]!=body:
                raise ValueError('Ambiguous reduced roller body mapping')
            if name in physical and name!=body:
                raise ValueError('Reduced roller descendant still has an independent physical body')
            reduced[name]=body;mapping[name]=body
    source=Path(bundle['srdf']);pairs=set();sources=[];coalesced=duplicates=collisionless=0
    for node in ET.parse(source).getroot().findall('disable_collisions'):
        a,b=node.get('link1'),node.get('link2')
        if a not in full or b not in full:
            raise ValueError('SRDF collision exclusion names a link outside generated URDF')
        if a not in mapping or b not in mapping:
            raise ValueError('SRDF collision exclusion has no physical body mapping')
        mapped=tuple(sorted((mapping[a],mapping[b])))
        sources.append(dict(links=[a,b],bodies=list(mapped),reason=node.get('reason','')))
        if mapped[0]==mapped[1]:
            coalesced+=1;continue
        if any(physical[name].find('collision') is None for name in mapped):
            collisionless+=1;continue
        if mapped in pairs:duplicates+=1
        pairs.add(mapped)
    return dict(srdf=str(source.resolve()),srdf_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                source_pair_count=len(sources),mapped_pair_count=len(pairs),
                coalesced_same_body_count=coalesced,duplicate_mapped_pair_count=duplicates,
                collisionless_pair_count=collisionless,pairs=[list(pair) for pair in sorted(pairs)],sources=sources)


def apply_srdf_collision_filters(stage,bundle,robot_root='/World/Robot'):
    """Author exact body pairs before physics initialization, preserving other targets."""
    from pxr import Sdf,UsdPhysics
    root=Sdf.Path(robot_root)
    if not root.IsAbsolutePath() or str(root)=='/':
        raise ValueError('An explicit robot root is required')
    plan=collision_filter_plan(bundle);allowed=set(_links(bundle['physics_urdf']))
    bodies={}
    for prim in stage.Traverse():
        if not prim.GetPath().HasPrefix(root) or not prim.HasAPI(UsdPhysics.RigidBodyAPI):continue
        name=prim.GetName()
        if name not in allowed:continue
        if name in bodies:raise ValueError('Ambiguous imported rigid body: '+name)
        bodies[name]=prim
    required={name for pair in plan['pairs'] for name in pair}
    missing=required-set(bodies)
    if missing:raise ValueError('Imported collision rigid bodies missing: '+', '.join(sorted(missing)))
    # Finish validating all source/target bodies before authoring any relationship.
    for name in required:
        if not UsdPhysics.FilteredPairsAPI.CanApply(bodies[name]):
            raise ValueError('Cannot filter imported rigid body: '+name)
    paths=[]
    for first,second in plan['pairs']:
        source,target=bodies[first],bodies[second]
        relation=UsdPhysics.FilteredPairsAPI.Apply(source).CreateFilteredPairsRel()
        if not relation.AddTarget(target.GetPath()):raise RuntimeError('Could not author SRDF collision exclusion')
        paths.append([str(source.GetPath()),str(target.GetPath())])
    return dict(plan,robot_root=str(root),rigid_body_count=len(bodies),applied_pair_count=len(paths),applied_pairs=paths)
