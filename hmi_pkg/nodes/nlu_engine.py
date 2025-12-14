#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
NLU (Natural Language Understanding) engine with phonetic matching
Preserves all phonetic corrections from original implementation
"""

import json
import re
import logging
from typing import Dict, Set, Optional, Tuple, List
from dataclasses import dataclass, field
from pathlib import Path
from config import Config

log = logging.getLogger("hmi.nlu")

def _edit_distance(a: str, b: str, max_cutoff: int = 3) -> int:
    """Calculate edit distance between two strings"""
    la, lb = len(a), len(b)
    if abs(la - lb) > max_cutoff:
        return max_cutoff + 1
    prev = list(range(lb + 1))
    for i in range(1, la + 1):
        cur = [i] + [0] * lb
        row_min = max_cutoff + 1
        ca = a[i - 1]
        for j in range(1, lb + 1):
            cost = 0 if ca == b[j - 1] else 1
            cur[j] = min(prev[j] + 1, cur[j - 1] + 1, prev[j - 1] + cost)
            if cur[j] < row_min: row_min = cur[j]
        if row_min > max_cutoff: return max_cutoff + 1
        prev = cur
    return prev[-1]

def _phonetic_similarity(a: str, b: str) -> float:
    """Calculate phonetic similarity between two words"""
    a, b = a.lower(), b.lower()
    if a == b: return 1.0
    
    phonetic_map = {'c':'k','k':'c','s':'z','z':'s','f':'v','v':'f','b':'p','p':'b','d':'t','t':'d','g':'k','x':'ks'}
    
    def normalize(w: str) -> str:
        w = w.replace('ck','k').replace('ph','f').replace('gh','f')
        return ''.join(phonetic_map.get(c,c) for c in w)
    
    a2, b2 = normalize(a), normalize(b)
    if a2 == b2: return 0.95
    
    dist = _edit_distance(a2, b2, max_cutoff=4)
    max_len = max(len(a2), len(b2)) or 1
    sim = 1.0 - (dist / max_len)
    
    if a and b and a[0] == b[0]: sim += 0.1
    
    aC = re.sub(r'[aeiou]', '', a)
    bC = re.sub(r'[aeiou]', '', b)
    if aC and bC:
        sim += (1.0 - (_edit_distance(aC,bC,3) / max(len(aC), len(bC)))) * 0.15
    
    return min(1.0, sim)

@dataclass
class NLU:
    """NLU model with phonetic matching support"""
    actions: Set[str]
    objects: Set[str]
    syn_act: Dict[str, str]
    syn_obj: Dict[str, str]
    act_aliases: Dict[str, Set[str]]
    obj_aliases: Dict[str, Set[str]]
    deny_action_tokens: Set[str] = (default_factory=lambda: {"please","pls","plz"})
    ACTION_PHONETIC_THRESHOLD: float = 0.65
    OBJECT_PHONETIC_THRESHOLD: float = 0.70

    @staticmethod
    def load(path: str) -> "NLU":
        """Load NLU model from capabilities file"""
        try:
            cap = json.load(open(path, "r", encoding="utf-8"))
        except Exception:
            cap = {"actions":["pick","release","next","stop","cancel"],
                   "objects":["cup","bottle","mug","can"],
                   "synonyms":{}}
        
        syn = cap.get("synonyms", {})
        syn_a_map, syn_o_map = {}, {}
        act_aliases, obj_aliases = {}, {}
        
        # Process base actions and objects
        for a in cap.get("actions", []):
            syn_a_map[a.lower()] = a
            act_aliases.setdefault(a.lower(), set()).add(a.lower())
        for o in cap.get("objects", []):
            syn_o_map[o.lower()] = o
            obj_aliases.setdefault(o.lower(), set()).add(o.lower())
        
        # Process synonyms from capabilities
        for k, lst in syn.get("actions", {}).items():
            for w in lst:
                syn_a_map[w.lower()] = k
                act_aliases.setdefault(k.lower(), set()).add(w.lower())
        for k, lst in syn.get("objects", {}).items():
            for w in lst:
                syn_o_map[w.lower()] = k
                obj_aliases.setdefault(k.lower(), set()).add(w.lower())
        
        # Apply soft phonetic corrections (domain knowledge)
        soft_act = {
            "pick": {"peak","pig","peek","ike","pik","pic","beak","pick up","grab","take"},
            "cancel": {"council","cancell","cansel","cancle","cancelled","counsel"},
            "release": {"releaze","relese","relees","rel ease","reliefs","drop"},
            "stop": {"top","spot","stock"},
            "next": {"necks","nest","text"}
        }
        for canon, alset in soft_act.items():
            for w in alset:
                syn_a_map.setdefault(w, canon)
                act_aliases.setdefault(canon, set()).add(w)
        
        soft_obj = {
            "can": {"ken","cane","aken","kan","a can"},
            "cup": {"cub","cap","kup"},
            "bottle": {"botto","bottal","batalla","bataille","a bottle","bottol","bottle."},
            "mug": {"mag","mog","mugg","mug."}
        }
        for canon, alset in soft_obj.items():
            for w in alset:
                syn_o_map.setdefault(w, canon)
                obj_aliases.setdefault(canon, set()).add(w)
        
        return NLU(set(cap.get("actions", [])), set(cap.get("objects", [])),
                   syn_a_map, syn_o_map, act_aliases, obj_aliases)

    def _compile_phrase(self, phrase: str) -> re.Pattern:
        p = re.sub(r"\s+", r"\\s+", re.escape(phrase.strip().lower()))
        return re.compile(rf"(?<!\w){p}(?!\w)")

    def _first_match(self, text: str, phrases: Set[str]) -> Optional[Tuple[str, int, int]]:
        for ph in sorted(phrases, key=lambda x: len(x), reverse=True):
            m = self._compile_phrase(ph).search(text)
            if m: return (ph, m.start(), m.end())
        return None

    def _fuzzy_to_action(self, token: str) -> Optional[str]:
        if token in self.deny_action_tokens: return None
        if token in self.syn_act:
            cand = self.syn_act[token]; return cand if cand in self.actions else None
        
        cand_keys = set(self.syn_act.keys()) | {a.lower() for a in self.actions}
        best_edit = (None, 999); best_phon = (None, 0.0)
        
        for k in cand_keys:
            d = _edit_distance(token, k, max_cutoff=3)
            if d < best_edit[1]: best_edit = (k, d)
            sim = _phonetic_similarity(token, k)
            if sim > best_phon[1]: best_phon = (k, sim)
        
        def canon(key): return self.syn_act.get(key, key) if key else None
        
        if best_phon[1] >= self.ACTION_PHONETIC_THRESHOLD:
            c = canon(best_phon[0]); return c if c in self.actions else None
        if best_edit[0]:
            c = canon(best_edit[0])
            if c in self.actions:
                if best_edit[1] <= 1: return c
                if c == "cancel" and best_edit[1] <= 3: return c
                if best_edit[1] <= 2: return c
        return None

    def _fuzzy_to_object(self, token: str) -> Optional[str]:
        if token in self.syn_obj:
            cand = self.syn_obj[token]; return cand if cand in self.objects else None
        
        cand_keys = set(self.syn_obj.keys()) | {o.lower() for o in self.objects}
        best_edit = (None, 999); best_phon = (None, 0.0)
        
        for k in cand_keys:
            d = _edit_distance(token, k, max_cutoff=3)
            if d < best_edit[1]: best_edit = (k, d)
            sim = _phonetic_similarity(token, k)
            if sim > best_phon[1]: best_phon = (k, sim)
        
        def canon(key): return self.syn_obj.get(key, key) if key else None
        
        if best_phon[1] >= self.OBJECT_PHONETIC_THRESHOLD:
            c = canon(best_phon[0]); return c if c in self.objects else None
        if best_edit[0] and best_edit[1] <= 2:
            c = canon(best_edit[0]); return c if c in self.objects else None
        return None

    def parse(self, text: str) -> Optional[Dict]:
        """Parse text to extract intent"""
        clean = re.sub(r"[^a-z0-9]+", " ", text.lower()).strip()
        if not clean: return None
        tokens: List[str] = clean.split()

        # phrase-level action first
        canon_action = None
        act_match = self._first_match(clean, set(self.syn_act.keys()))
        if act_match:
            alias, _, _ = act_match
            cand = self.syn_act.get(alias, alias)
            if cand in self.actions: canon_action = cand

        if not canon_action:
            for t in tokens:
                ca = self._fuzzy_to_action(t)
                if ca and ca in self.actions: canon_action = ca; break

        canon_object = None
        if canon_action and canon_action == "pick":
            m = self._first_match(clean, {"pick","pick up"})
            start = m[2] if m else 0
            post = clean[start:].strip()
            om = self._first_match(post, set(self.syn_obj.keys()))
            if om:
                alias2, _, _ = om
                co = self.syn_obj.get(alias2, alias2)
                if co in self.objects: canon_object = co
            if not canon_object:
                for t in post.split():
                    co = self._fuzzy_to_object(t)
                    if co and co in self.objects: canon_object = co; break

        if not canon_object:
            om2 = self._first_match(clean, set(self.syn_obj.keys()))
            if om2:
                alias3, _, _ = om2
                co = self.syn_obj.get(alias3, alias3)
                if co in self.objects: canon_object = co

        if not canon_object:
            for t in tokens:
                co = self._fuzzy_to_object(t)
                if co and co in self.objects: canon_object = co; break

        if canon_action or canon_object:
            return {"action": canon_action, "object": canon_object}
        return None

def load_capabilities():
    """Load capabilities and create NLU model"""
    nlu_model = NLU.load(Config.CAPABILITIES_PATH)
    
    # Apply threshold configurations
    try:
        nlu_model.ACTION_PHONETIC_THRESHOLD = Config.NLU_ACTION_THRESHOLD
        nlu_model.OBJECT_PHONETIC_THRESHOLD = Config.NLU_OBJECT_THRESHOLD
    except Exception:
        pass
    
    log.info(f"Loaded NLU: actions={nlu_model.actions}, objects={nlu_model.objects}")
    return nlu_model

def load_capabilities_for_prompt() -> Tuple[List[str], List[str]]:
    """Load just actions and objects for Whisper prompt"""
    try:
        j = json.load(open(Config.CAPABILITIES_PATH, "r", encoding="utf-8"))
        return list(j.get("actions", [])), list(j.get("objects", []))
    except Exception:
        return [], []
