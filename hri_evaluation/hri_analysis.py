#!/usr/bin/env python3
"""
================================================================================
HRI QUESTIONNAIRE ANALYSIS SCRIPT
Dum-E Robot Interaction Study - Hypothesis Testing
================================================================================

USAGE:
    python hri_analysis.py <path_to_excel_file.xlsx>

HYPOTHESES:
    H1: Cognitive load difference between modalities (paired, n=both-mode testers)
    H2: SUS difference by preference (all participants)
    H3: Learnability correlates with technical background (all participants)
    H4: Control-seeking users prefer on-demand (on-demand testers)
    H5: Safety perception predicts sequence preference (all participants)

================================================================================
"""

import pandas as pd
import numpy as np
from scipy import stats
from scipy.stats import shapiro, mannwhitneyu, wilcoxon, spearmanr, pearsonr, pointbiserialr
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
import warnings
import sys
import json
from datetime import datetime
from io import StringIO

warnings.filterwarnings('ignore')


# =============================================================================
# COLUMN INDEX DEFINITIONS
# =============================================================================

COL_AGE = 3
COL_GENDER = 4
COL_TECH_EXPERTISE = 5
COL_TECH_MOVEMENT = 6
COL_TECH_PROXIMITY = 7
COL_TECH_SENSORS = 8
COL_TECH_CV = 9
COL_TECH_ROS = 10
COL_TECH_COBOT = 11
COL_TECH_TELEOP = 12

COL_DOF_QUESTION = 13
DOF_CORRECT_ANSWER = "independent joints or axes"

# Godspeed Safety (cols O-R, indices 14-17)
COL_GODSPEED_RELAXED = 14
COL_GODSPEED_CALM = 15
COL_GODSPEED_BOLD = 16
COL_GODSPEED_SAFE = 17

# Safety perception (cols S-V, indices 18-21)
COL_SAFETY_SPEED = 18
COL_SAFETY_COMFORT = 19
COL_SAFETY_AWARENESS = 20
COL_SAFETY_PREDICTABLE = 21

COL_MODE = 22

# Sequence mode - single mode testers
COL_SEQ_ATTENTION_SINGLE = 23
COL_SEQ_TIMING_SINGLE = 24
COL_SEQ_TRUST_SINGLE = 25
COL_SEQ_SAFETY_SINGLE = 26

# Sequence mode - both mode testers
COL_SEQ_ATTENTION_BOTH = 32
COL_SEQ_TIMING_BOTH = 33
COL_SEQ_TRUST_BOTH = 34
COL_SEQ_SAFETY_BOTH = 35

# On-demand mode - both mode testers
COL_OND_ATTENTION_BOTH = 36
COL_OND_UNDERSTANDING_BOTH = 37
COL_OND_CONTROL_BOTH = 38
COL_OND_TIMING_BOTH = 39
COL_OND_EXHAUSTION_BOTH = 40

COL_EFFICIENCY_PREF = 41
COL_DAILY_PREF = 42
COL_SUS_START = 43


# =============================================================================
# UTILITY FUNCTIONS
# =============================================================================

def parse_cell_value(value):
    """Parse cell values, averaging multiple comma-separated values."""
    if pd.isna(value):
        return np.nan
    str_val = str(value).strip()
    if ',' in str_val:
        try:
            parts = [float(x.strip()) for x in str_val.split(',') if x.strip()]
            return np.mean(parts)
        except ValueError:
            return np.nan
    else:
        try:
            return float(str_val)
        except ValueError:
            return np.nan


def check_dof_answer(answer_str):
    """Check if DoF answer is correct (1) or incorrect (0)."""
    if pd.isna(answer_str):
        return 0
    answer_lower = str(answer_str).lower()
    if "independent" in answer_lower and ("joint" in answer_lower or "axes" in answer_lower):
        return 1
    return 0


def compute_cronbach_alpha(item_matrix, report_log=None, scale_name=""):
    """
    Compute Cronbach's alpha for internal consistency.
    
    Formula: α = (k/(k-1)) × (1 - Σσᵢ²/σₜ²)
    """
    item_matrix = item_matrix.dropna()
    
    if len(item_matrix) < 3:
        if report_log:
            report_log(f"    ⚠ Insufficient data for {scale_name} (n={len(item_matrix)})")
        return None
    
    k = item_matrix.shape[1]
    if k < 2:
        if report_log:
            report_log(f"    ⚠ Need at least 2 items for {scale_name}")
        return None
    
    item_vars = item_matrix.var(axis=0, ddof=1)
    total_var = item_matrix.sum(axis=1).var(ddof=1)
    
    if total_var == 0:
        if report_log:
            report_log(f"    ⚠ Zero variance for {scale_name}")
        return None
    
    alpha = (k / (k - 1)) * (1 - item_vars.sum() / total_var)
    
    if alpha >= 0.9:
        interp = "Excellent"
    elif alpha >= 0.8:
        interp = "Good"
    elif alpha >= 0.7:
        interp = "Acceptable"
    elif alpha >= 0.6:
        interp = "Questionable"
    else:
        interp = "Poor"
    
    if report_log:
        report_log(f"    {scale_name}: α = {alpha:.4f} ({interp})")
        report_log(f"      k={k} items, Σσᵢ²={item_vars.sum():.4f}, σₜ²={total_var:.4f}")
    
    return {'alpha': float(alpha), 'interpretation': interp, 'k': k}


# =============================================================================
# DATA EXTRACTION
# =============================================================================

class HRIDataExtractor:
    def __init__(self, filepath: str):
        self.filepath = filepath
        self.raw_df = pd.read_excel(filepath)
        self.n_participants = len(self.raw_df)
        self.data = None
        self.report = StringIO()
        
    def log(self, message: str):
        print(message)
        self.report.write(message + "\n")
    
    def extract_all(self) -> pd.DataFrame:
        self.log("\n" + "="*70)
        self.log("STEP 1: DATA EXTRACTION")
        self.log("="*70)
        self.log(f"\nSource file: {self.filepath}")
        self.log(f"Total participants: {self.n_participants}")
        
        participants = []
        
        # Multi-value detection
        self.log("\n--- 1.1 Multi-Value Cell Detection ---")
        for col_idx in range(len(self.raw_df.columns)):
            for row_idx in range(len(self.raw_df)):
                val = str(self.raw_df.iloc[row_idx, col_idx])
                if ',' in val and len(val) < 15:
                    avg = parse_cell_value(val)
                    self.log(f"  P{row_idx}, Col {col_idx}: '{val}' → average = {avg:.2f}")
        
        # DoF Question
        self.log("\n--- 1.2 Degrees of Freedom Question ---")
        for idx in range(self.n_participants):
            answer = self.raw_df.iloc[idx, COL_DOF_QUESTION]
            is_correct = check_dof_answer(answer)
            status = "✓ CORRECT" if is_correct else "✗ INCORRECT"
            self.log(f"  P{idx}: {status}")
        
        # Mode Assignment
        self.log("\n--- 1.3 Testing Mode Assignment ---")
        
        for idx in range(self.n_participants):
            row = self.raw_df.iloc[idx]
            mode_str = str(row.iloc[COL_MODE])
            
            tested_sequence = False
            tested_ondemand = False
            tested_both = False
            
            if "First" in mode_str and "then" in mode_str:
                tested_both = True
                tested_sequence = True
                tested_ondemand = True
            elif "Sequence" in mode_str:
                tested_sequence = True
            elif "On Demand" in mode_str or "On-demand" in mode_str:
                tested_ondemand = True
            
            self.log(f"  P{idx}: Seq={tested_sequence}, OnD={tested_ondemand}, Both={tested_both}")
            
            p = {
                'participant_id': idx,
                'age': parse_cell_value(row.iloc[COL_AGE]),
                'gender': row.iloc[COL_GENDER],
                'testing_mode': mode_str,
                'tested_sequence': tested_sequence,
                'tested_ondemand': tested_ondemand,
                'tested_both': tested_both,
            }
            
            # Technical Background
            tech_items = [
                parse_cell_value(row.iloc[COL_TECH_EXPERTISE]),
                parse_cell_value(row.iloc[COL_TECH_MOVEMENT]),
                parse_cell_value(row.iloc[COL_TECH_SENSORS]),
                parse_cell_value(row.iloc[COL_TECH_CV]),
                parse_cell_value(row.iloc[COL_TECH_ROS]),
                parse_cell_value(row.iloc[COL_TECH_COBOT]),
                parse_cell_value(row.iloc[COL_TECH_TELEOP]),
            ]
            
            # Store individual items for Cronbach's alpha
            p['tech_item_1'] = tech_items[0]
            p['tech_item_2'] = tech_items[1]
            p['tech_item_3'] = tech_items[2]
            p['tech_item_4'] = tech_items[3]
            p['tech_item_5'] = tech_items[4]
            p['tech_item_6'] = tech_items[5]
            p['tech_item_7'] = tech_items[6]
            
            dof_correct = check_dof_answer(row.iloc[COL_DOF_QUESTION])
            
            p['tech_self_assess_mean'] = np.nanmean(tech_items)
            p['tech_dof_correct'] = dof_correct
            p['tech_composite'] = 0.7 * p['tech_self_assess_mean'] + 0.3 * dof_correct * 5
            
            # Godspeed Safety Scale
            godspeed_items = [
                parse_cell_value(row.iloc[COL_GODSPEED_RELAXED]),
                parse_cell_value(row.iloc[COL_GODSPEED_CALM]),
                parse_cell_value(row.iloc[COL_GODSPEED_BOLD]),
                parse_cell_value(row.iloc[COL_GODSPEED_SAFE]),
            ]
            p['godspeed_relaxed'] = godspeed_items[0]
            p['godspeed_calm'] = godspeed_items[1]
            p['godspeed_bold'] = godspeed_items[2]
            p['godspeed_safe'] = godspeed_items[3]
            p['godspeed_safety_mean'] = np.nanmean(godspeed_items)
            
            # Safety Perception Scale (for H5)
            p['safety_speed'] = parse_cell_value(row.iloc[COL_SAFETY_SPEED])
            p['safety_comfort'] = parse_cell_value(row.iloc[COL_SAFETY_COMFORT])
            p['safety_awareness'] = parse_cell_value(row.iloc[COL_SAFETY_AWARENESS])
            p['safety_predictable'] = parse_cell_value(row.iloc[COL_SAFETY_PREDICTABLE])
            p['safety_perception_mean'] = np.nanmean([
                p['safety_speed'], p['safety_comfort'], 
                p['safety_awareness'], p['safety_predictable']
            ])
            
            # Sequence mode data
            if tested_both:
                p['seq_attention'] = parse_cell_value(row.iloc[COL_SEQ_ATTENTION_BOTH])
                p['seq_timing'] = parse_cell_value(row.iloc[COL_SEQ_TIMING_BOTH])
                p['seq_trust'] = parse_cell_value(row.iloc[COL_SEQ_TRUST_BOTH])
                p['seq_safety'] = parse_cell_value(row.iloc[COL_SEQ_SAFETY_BOTH])
            elif tested_sequence:
                p['seq_attention'] = parse_cell_value(row.iloc[COL_SEQ_ATTENTION_SINGLE])
                p['seq_timing'] = parse_cell_value(row.iloc[COL_SEQ_TIMING_SINGLE])
                p['seq_trust'] = parse_cell_value(row.iloc[COL_SEQ_TRUST_SINGLE])
                p['seq_safety'] = parse_cell_value(row.iloc[COL_SEQ_SAFETY_SINGLE])
            else:
                p['seq_attention'] = np.nan
                p['seq_timing'] = np.nan
                p['seq_trust'] = np.nan
                p['seq_safety'] = np.nan
            
            # On-demand mode data
            if tested_both:
                p['ond_attention'] = parse_cell_value(row.iloc[COL_OND_ATTENTION_BOTH])
                p['ond_understanding'] = parse_cell_value(row.iloc[COL_OND_UNDERSTANDING_BOTH])
                p['ond_control'] = parse_cell_value(row.iloc[COL_OND_CONTROL_BOTH])
                p['ond_timing'] = parse_cell_value(row.iloc[COL_OND_TIMING_BOTH])
                p['ond_exhaustion'] = parse_cell_value(row.iloc[COL_OND_EXHAUSTION_BOTH])
            else:
                p['ond_attention'] = np.nan
                p['ond_understanding'] = np.nan
                p['ond_control'] = np.nan
                p['ond_timing'] = np.nan
                p['ond_exhaustion'] = np.nan
            
            # Preferences
            p['efficiency_pref'] = str(row.iloc[COL_EFFICIENCY_PREF]).lower().strip()
            p['daily_pref'] = str(row.iloc[COL_DAILY_PREF]).lower().strip()
            
            # SUS items
            for i in range(9):
                p[f'sus_raw_{i+1}'] = parse_cell_value(row.iloc[COL_SUS_START + i])
            
            participants.append(p)
        
        self.data = pd.DataFrame(participants)
        
        # Technical Background Summary
        self.log("\n--- 1.4 Technical Background Computation ---")
        self.log("  Formula: tech_composite = 0.7 × self_assess + 0.3 × dof × 5")
        for idx in range(len(self.data)):
            row = self.data.iloc[idx]
            self.log(f"  P{idx}: {row['tech_self_assess_mean']:.2f} × 0.7 + {int(row['tech_dof_correct'])} × 1.5 = {row['tech_composite']:.2f}")
        
        # Summary
        self.log(f"\n--- 1.5 Summary ---")
        self.log(f"  Total: {self.n_participants}")
        self.log(f"  Tested Sequence: {self.data['tested_sequence'].sum()}")
        self.log(f"  Tested On-demand: {self.data['tested_ondemand'].sum()}")
        self.log(f"  Tested BOTH: {self.data['tested_both'].sum()}")
        self.log(f"  DoF correct: {int(self.data['tech_dof_correct'].sum())}/{self.n_participants}")
        
        return self.data
    
    def get_report(self) -> str:
        return self.report.getvalue()


# =============================================================================
# SCORE COMPUTATIONS
# =============================================================================

def compute_sus_scores(data: pd.DataFrame, report_log) -> pd.DataFrame:
    report_log("\n" + "="*70)
    report_log("STEP 2: SUS SCORE COMPUTATION")
    report_log("="*70)
    
    data = data.copy()
    polarity = {1: 'pos', 2: 'neg', 3: 'pos', 4: 'neg', 5: 'pos', 
                6: 'neg', 7: 'pos', 8: 'neg', 9: 'pos'}
    
    report_log("\n  Scoring: Positive items: score = raw - 1")
    report_log("           Negative items: score = 5 - raw")
    
    item_scores = []
    for q in range(1, 10):
        raw_col = f'sus_raw_{q}'
        score_col = f'sus_score_{q}'
        if polarity[q] == 'pos':
            data[score_col] = data[raw_col] - 1
        else:
            data[score_col] = 5 - data[raw_col]
        item_scores.append(score_col)
    
    data['sus_sum'] = data[item_scores].sum(axis=1)
    multiplier = 100 / 36
    data['sus_total'] = data['sus_sum'] * multiplier
    
    report_log(f"\n  SUS = sum × {multiplier:.4f} (adjusted for 9 items)")
    
    for idx in range(len(data)):
        sus_total = data.iloc[idx]['sus_total']
        report_log(f"  P{idx}: SUS = {sus_total:.2f}")
    
    return data


def compute_learnability_scores(data: pd.DataFrame, report_log) -> pd.DataFrame:
    report_log("\n" + "="*70)
    report_log("STEP 3: LEARNABILITY COMPUTATION")
    report_log("="*70)
    
    data = data.copy()
    report_log("\n  Formula: learnability = mean(Q7_score, Q9_score) × 25")
    
    data['learnability'] = (data['sus_score_7'] + data['sus_score_9']) / 2 * 25
    
    for idx in range(len(data)):
        s7 = data.iloc[idx]['sus_score_7']
        s9 = data.iloc[idx]['sus_score_9']
        learn = data.iloc[idx]['learnability']
        report_log(f"  P{idx}: ({int(s7)} + {int(s9)})/2 × 25 = {learn:.2f}")
    
    return data


def add_preference_categories(data: pd.DataFrame) -> pd.DataFrame:
    """Add binary preference indicators."""
    data = data.copy()
    
    def categorize_pref(pref):
        pref = str(pref).lower()
        if 'on-demand' in pref or 'on demand' in pref:
            return 'On-demand'
        elif 'sequence' in pref:
            return 'Sequence'
        return 'Other'
    
    data['pref_category'] = data['daily_pref'].apply(categorize_pref)
    data['prefers_ondemand'] = (data['pref_category'] == 'On-demand').astype(int)
    data['prefers_sequence'] = (data['pref_category'] == 'Sequence').astype(int)
    
    return data


# =============================================================================
# HYPOTHESIS ANALYSES
# =============================================================================

def analyze_h1(data: pd.DataFrame, report_log) -> dict:
    """H1: Cognitive Load Difference (paired comparison, both-mode testers only)"""
    
    report_log("\n" + "="*70)
    report_log("HYPOTHESIS 1: COGNITIVE LOAD DIFFERENCE")
    report_log("="*70)
    report_log("\n  H1: Sequence mode results in lower cognitive load than On-demand")
    report_log("  Sample: Only participants who tested BOTH modes (paired design)")
    
    results = {'hypothesis': 'H1', 'significant': False}
    
    # All available data
    seq_mask = data['tested_sequence'] == True
    seq_attention_all = data.loc[seq_mask, 'seq_attention'].dropna()
    
    ond_mask = data['tested_ondemand'] == True
    ond_attention_all = data.loc[ond_mask, 'ond_attention'].dropna()
    ond_exhaustion_all = data.loc[ond_mask, 'ond_exhaustion'].dropna()
    
    report_log(f"\n--- Descriptive Statistics (ALL available) ---")
    report_log(f"  Sequence Attention (n={len(seq_attention_all)}): {[round(v,1) for v in seq_attention_all.tolist()]}")
    report_log(f"    M={seq_attention_all.mean():.2f}, SD={seq_attention_all.std():.2f}")
    report_log(f"  On-demand Attention (n={len(ond_attention_all)}): {[round(v,1) for v in ond_attention_all.tolist()]}")
    report_log(f"    M={ond_attention_all.mean():.2f}, SD={ond_attention_all.std():.2f}")
    report_log(f"  On-demand Exhaustion (n={len(ond_exhaustion_all)}): {[round(v,1) for v in ond_exhaustion_all.tolist()]}")
    report_log(f"    M={ond_exhaustion_all.mean():.2f}, SD={ond_exhaustion_all.std():.2f}")
    
    results['seq_attention'] = {'n': len(seq_attention_all), 'mean': float(seq_attention_all.mean()), 
                                'sd': float(seq_attention_all.std()), 'values': seq_attention_all.tolist()}
    results['ond_attention'] = {'n': len(ond_attention_all), 'mean': float(ond_attention_all.mean()),
                                'sd': float(ond_attention_all.std()), 'values': ond_attention_all.tolist()}
    
    # Paired comparison
    both_mask = data['tested_both'] == True
    paired_data = data[both_mask].copy()
    n_paired = len(paired_data)
    
    report_log(f"\n--- Paired Comparison (n={n_paired}) ---")
    
    if n_paired >= 3:
        seq_paired = paired_data['seq_attention'].values
        ond_paired = paired_data['ond_attention'].values
        diff = seq_paired - ond_paired
        
        report_log(f"  Seq:  {[round(v,1) for v in seq_paired]}")
        report_log(f"  OnD:  {[round(v,1) for v in ond_paired]}")
        report_log(f"  Diff: {[round(v,1) for v in diff]} (Mean={np.mean(diff):.2f})")
        
        results['paired_n'] = n_paired
        results['mean_diff'] = float(np.mean(diff))
        
        try:
            stat_w, p_w = wilcoxon(seq_paired, ond_paired, alternative='two-sided')
            z_score = stats.norm.ppf(p_w / 2) if p_w < 1 else 0
            effect_r = abs(z_score) / np.sqrt(n_paired)
            
            report_log(f"\n  Wilcoxon: W={stat_w:.2f}, p={p_w:.4f}, r={effect_r:.3f}")
            results['wilcoxon'] = {'W': float(stat_w), 'p': float(p_w), 'effect_r': float(effect_r)}
            
            if p_w < 0.05:
                results['significant'] = True
                report_log(f"  → SIGNIFICANT at α=0.05")
            else:
                report_log(f"  → NOT SIGNIFICANT at α=0.05")
        except Exception as e:
            report_log(f"  ⚠ Test error: {e}")
    
    return results


def analyze_h2(data: pd.DataFrame, report_log) -> dict:
    """H2: SUS Difference by Preference (all participants)"""
    
    report_log("\n" + "="*70)
    report_log("HYPOTHESIS 2: SUS BY PREFERENCE")
    report_log("="*70)
    report_log("\n  H2: SUS scores differ between preference groups")
    report_log("  Sample: ALL participants (n={})".format(len(data)))
    report_log("\n  ⚠ Note: SUS administered once, not per-mode")
    
    results = {'hypothesis': 'H2', 'significant': False}
    
    sus_all = data['sus_total'].dropna()
    
    report_log(f"\n--- Overall SUS (n={len(sus_all)}) ---")
    report_log(f"  Values: {[round(v,1) for v in sus_all.tolist()]}")
    report_log(f"  M={sus_all.mean():.2f}, SD={sus_all.std():.2f}")
    
    mean_sus = sus_all.mean()
    if mean_sus >= 85.5: grade = "A (Excellent)"
    elif mean_sus >= 72.6: grade = "B (Good)"
    elif mean_sus >= 52.0: grade = "C (OK)"
    else: grade = "D/F (Poor)"
    
    report_log(f"  Grade: {grade}")
    results['sus_overall'] = {'n': len(sus_all), 'mean': float(mean_sus), 
                              'sd': float(sus_all.std()), 'grade': grade}
    
    # By preference
    ond_pref = data[data['pref_category'] == 'On-demand']['sus_total'].dropna()
    seq_pref = data[data['pref_category'] == 'Sequence']['sus_total'].dropna()
    
    report_log(f"\n--- By Preference ---")
    report_log(f"  On-demand pref (n={len(ond_pref)}): {[round(v,1) for v in ond_pref.tolist()]}")
    if len(ond_pref) > 0:
        report_log(f"    M={ond_pref.mean():.2f}, SD={ond_pref.std():.2f}")
    report_log(f"  Sequence pref (n={len(seq_pref)}): {[round(v,1) for v in seq_pref.tolist()]}")
    if len(seq_pref) > 0:
        report_log(f"    M={seq_pref.mean():.2f}")
    
    if len(ond_pref) >= 2 and len(seq_pref) >= 2:
        stat_u, p_mw = mannwhitneyu(ond_pref, seq_pref, alternative='two-sided')
        report_log(f"\n  Mann-Whitney U: U={stat_u:.2f}, p={p_mw:.4f}")
        results['mann_whitney'] = {'U': float(stat_u), 'p': float(p_mw)}
        if p_mw < 0.05:
            results['significant'] = True
    else:
        report_log(f"\n  ⚠ Cannot test: need n≥2 in each group")
        results['error'] = 'Insufficient group sizes'
    
    return results


def analyze_h3(data: pd.DataFrame, report_log) -> dict:
    """H3: Learnability correlates with Technical Background (ALL participants)"""
    
    report_log("\n" + "="*70)
    report_log("HYPOTHESIS 3: LEARNABILITY VS TECHNICAL BACKGROUND")
    report_log("="*70)
    report_log("\n  H3: Users with lower tech background report lower learnability")
    report_log("  Sample: ALL participants (tech background measured for everyone)")
    
    results = {'hypothesis': 'H3', 'significant': False}
    
    # Use ALL participants
    valid_mask = data['tech_composite'].notna() & data['learnability'].notna()
    valid_data = data[valid_mask]
    n_valid = len(valid_data)
    
    report_log(f"\n--- Data (n={n_valid}, ALL participants) ---")
    
    tech_vals = valid_data['tech_composite'].values
    learn_vals = valid_data['learnability'].values
    
    report_log(f"  Technical Background: {[round(v,2) for v in tech_vals]}")
    report_log(f"    M={np.mean(tech_vals):.2f}, SD={np.std(tech_vals):.2f}")
    report_log(f"  Learnability: {[round(v,1) for v in learn_vals]}")
    report_log(f"    M={np.mean(learn_vals):.2f}, SD={np.std(learn_vals):.2f}")
    
    results['tech_background'] = {'n': n_valid, 'mean': float(np.mean(tech_vals)), 
                                  'sd': float(np.std(tech_vals)), 'values': tech_vals.tolist()}
    results['learnability'] = {'n': n_valid, 'mean': float(np.mean(learn_vals)),
                               'sd': float(np.std(learn_vals)), 'values': learn_vals.tolist()}
    
    report_log(f"\n  Paired values:")
    for i, row in valid_data.iterrows():
        dof = "✓" if row['tech_dof_correct'] == 1 else "✗"
        report_log(f"    P{int(row['participant_id'])}: Tech={row['tech_composite']:.2f} (DoF {dof}), Learn={row['learnability']:.1f}")
    
    if n_valid >= 3:
        rho, p_spearman = spearmanr(tech_vals, learn_vals)
        
        abs_rho = abs(rho)
        if abs_rho >= 0.7: strength = "strong"
        elif abs_rho >= 0.4: strength = "moderate"
        elif abs_rho >= 0.2: strength = "weak"
        else: strength = "negligible"
        direction = "positive" if rho > 0 else "negative"
        
        report_log(f"\n--- Spearman Correlation ---")
        report_log(f"  ρ = {rho:.4f}, p = {p_spearman:.4f}")
        report_log(f"  Interpretation: {strength} {direction}")
        
        results['spearman'] = {'rho': float(rho), 'p': float(p_spearman), 
                              'strength': strength, 'direction': direction}
        
        if p_spearman < 0.05:
            results['significant'] = True
            report_log(f"  → SIGNIFICANT at α=0.05")
        else:
            report_log(f"  → NOT SIGNIFICANT at α=0.05")
    
    return results


def analyze_h4(data: pd.DataFrame, report_log) -> dict:
    """H4: Control-seeking users prefer On-demand (on-demand testers only)"""
    
    report_log("\n" + "="*70)
    report_log("HYPOTHESIS 4: CONTROL-SEEKING → ON-DEMAND PREFERENCE")
    report_log("="*70)
    report_log("\n  H4: High control scores predict on-demand preference")
    report_log("  Sample: Participants who tested On-demand mode")
    
    results = {'hypothesis': 'H4', 'significant': False}
    
    # Only participants who tested on-demand (have control ratings)
    ond_testers = data[data['tested_ondemand'] == True].copy()
    valid_mask = ond_testers['ond_control'].notna()
    valid_data = ond_testers[valid_mask]
    n_valid = len(valid_data)
    
    report_log(f"\n--- Data (n={n_valid}, on-demand testers) ---")
    
    if n_valid < 3:
        report_log(f"  ⚠ Insufficient data")
        results['error'] = 'Insufficient data'
        return results
    
    control_vals = valid_data['ond_control'].values
    prefers_ond = valid_data['prefers_ondemand'].values
    
    report_log(f"  Control scores: {[round(v,1) for v in control_vals]}")
    report_log(f"  Prefers On-demand: {prefers_ond.tolist()}")
    
    report_log(f"\n  Individual data:")
    for _, row in valid_data.iterrows():
        pref = "On-demand" if row['prefers_ondemand'] == 1 else "Sequence/Other"
        report_log(f"    P{int(row['participant_id'])}: Control={row['ond_control']:.1f}, Pref={pref}")
    
    results['control'] = {'n': n_valid, 'mean': float(np.mean(control_vals)),
                         'sd': float(np.std(control_vals)), 'values': control_vals.tolist()}
    
    # Point-biserial correlation (continuous vs binary)
    try:
        r_pb, p_pb = pointbiserialr(prefers_ond, control_vals)
        
        report_log(f"\n--- Point-Biserial Correlation ---")
        report_log(f"  r = {r_pb:.4f}, p = {p_pb:.4f}")
        
        results['point_biserial'] = {'r': float(r_pb), 'p': float(p_pb)}
        
        if p_pb < 0.05:
            results['significant'] = True
            report_log(f"  → SIGNIFICANT at α=0.05")
        else:
            report_log(f"  → NOT SIGNIFICANT at α=0.05")
            
        # Group comparison
        ond_pref_control = valid_data[valid_data['prefers_ondemand'] == 1]['ond_control']
        other_pref_control = valid_data[valid_data['prefers_ondemand'] == 0]['ond_control']
        
        report_log(f"\n--- Group Comparison ---")
        report_log(f"  On-demand pref (n={len(ond_pref_control)}): M={ond_pref_control.mean():.2f}")
        if len(other_pref_control) > 0:
            report_log(f"  Other pref (n={len(other_pref_control)}): M={other_pref_control.mean():.2f}")
        else:
            report_log(f"  Other pref (n=0): N/A")
        
    except Exception as e:
        report_log(f"  ⚠ Test error: {e}")
        results['error'] = str(e)
    
    return results


def analyze_h5(data: pd.DataFrame, report_log) -> dict:
    """H5: Safety perception predicts Sequence preference (all participants)"""
    
    report_log("\n" + "="*70)
    report_log("HYPOTHESIS 5: SAFETY PERCEPTION → SEQUENCE PREFERENCE")
    report_log("="*70)
    report_log("\n  H5: High safety ratings predict sequence preference")
    report_log("  Items: speed appropriate, robot awareness, trust to complete")
    report_log("  Sample: ALL participants")
    
    results = {'hypothesis': 'H5', 'significant': False}
    
    # Compute safety composite for H5 (specific items)
    # "speed felt appropriate and safe" = safety_speed
    # "robot appeared aware of my presence" = safety_awareness
    # "I trusted the robot to complete without hitting" = seq_trust (for sequence testers)
    
    # For all participants, use available safety items
    valid_data = data.copy()
    
    # Create H5 safety composite from available items
    # Note: seq_trust only available for sequence testers
    valid_data['h5_safety_composite'] = valid_data[['safety_speed', 'safety_awareness']].mean(axis=1)
    
    # Include seq_trust where available
    seq_testers = valid_data['tested_sequence'] == True
    valid_data.loc[seq_testers, 'h5_safety_composite'] = valid_data.loc[seq_testers, 
        ['safety_speed', 'safety_awareness', 'seq_trust']].mean(axis=1)
    
    valid_mask = valid_data['h5_safety_composite'].notna()
    valid_data = valid_data[valid_mask]
    n_valid = len(valid_data)
    
    report_log(f"\n--- Data (n={n_valid}, all participants) ---")
    
    safety_vals = valid_data['h5_safety_composite'].values
    prefers_seq = valid_data['prefers_sequence'].values
    
    report_log(f"  Safety composite: {[round(v,2) for v in safety_vals]}")
    report_log(f"  Prefers Sequence: {prefers_seq.tolist()}")
    
    report_log(f"\n  Individual data:")
    for _, row in valid_data.iterrows():
        pref = "Sequence" if row['prefers_sequence'] == 1 else "On-demand/Other"
        report_log(f"    P{int(row['participant_id'])}: Safety={row['h5_safety_composite']:.2f}, Pref={pref}")
    
    results['safety'] = {'n': n_valid, 'mean': float(np.mean(safety_vals)),
                        'sd': float(np.std(safety_vals)), 'values': safety_vals.tolist()}
    
    if n_valid >= 3:
        try:
            r_pb, p_pb = pointbiserialr(prefers_seq, safety_vals)
            
            report_log(f"\n--- Point-Biserial Correlation ---")
            report_log(f"  r = {r_pb:.4f}, p = {p_pb:.4f}")
            
            results['point_biserial'] = {'r': float(r_pb), 'p': float(p_pb)}
            
            if p_pb < 0.05:
                results['significant'] = True
                report_log(f"  → SIGNIFICANT at α=0.05")
            else:
                report_log(f"  → NOT SIGNIFICANT at α=0.05")
                
            # Group comparison
            seq_pref_safety = valid_data[valid_data['prefers_sequence'] == 1]['h5_safety_composite']
            other_pref_safety = valid_data[valid_data['prefers_sequence'] == 0]['h5_safety_composite']
            
            report_log(f"\n--- Group Comparison ---")
            if len(seq_pref_safety) > 0:
                report_log(f"  Sequence pref (n={len(seq_pref_safety)}): M={seq_pref_safety.mean():.2f}")
            if len(other_pref_safety) > 0:
                report_log(f"  Other pref (n={len(other_pref_safety)}): M={other_pref_safety.mean():.2f}")
                
        except Exception as e:
            report_log(f"  ⚠ Test error: {e}")
            results['error'] = str(e)
    
    return results


# =============================================================================
# RELIABILITY ANALYSIS
# =============================================================================

def analyze_reliability(data: pd.DataFrame, report_log) -> dict:
    """Compute Cronbach's alpha for all multi-item scales."""
    
    report_log("\n" + "="*70)
    report_log("RELIABILITY ANALYSIS (CRONBACH'S ALPHA)")
    report_log("="*70)
    report_log("\n  Formula: α = (k/(k-1)) × (1 - Σσᵢ²/σₜ²)")
    
    results = {}
    
    # SUS Scale (9 items)
    report_log("\n--- H2: SUS Scale ---")
    sus_cols = [f'sus_score_{i}' for i in range(1, 10)]
    sus_result = compute_cronbach_alpha(data[sus_cols], report_log, "SUS (9 items)")
    if sus_result:
        results['sus'] = sus_result
    
    # Technical Background Scale (7 items)
    report_log("\n--- H3: Technical Background Scale ---")
    tech_cols = [f'tech_item_{i}' for i in range(1, 8)]
    tech_result = compute_cronbach_alpha(data[tech_cols], report_log, "Tech Background (7 items)")
    if tech_result:
        results['tech_background'] = tech_result
    
    # Godspeed Safety Scale (4 items)
    report_log("\n--- Godspeed Safety Scale ---")
    godspeed_cols = ['godspeed_relaxed', 'godspeed_calm', 'godspeed_bold', 'godspeed_safe']
    godspeed_result = compute_cronbach_alpha(data[godspeed_cols], report_log, "Godspeed Safety (4 items)")
    if godspeed_result:
        results['godspeed_safety'] = godspeed_result
    
    # Safety Perception Scale (4 items)
    report_log("\n--- H5: Safety Perception Scale ---")
    safety_cols = ['safety_speed', 'safety_comfort', 'safety_awareness', 'safety_predictable']
    safety_result = compute_cronbach_alpha(data[safety_cols], report_log, "Safety Perception (4 items)")
    if safety_result:
        results['safety_perception'] = safety_result
    
    # H1 Cognitive Load items (if applicable)
    report_log("\n--- H1: Mode-specific items ---")
    report_log("    Note: Attention items are single-item measures, α not applicable")
    
    # H4 Control item
    report_log("\n--- H4: Control item ---")
    report_log("    Note: Single-item measure, α not applicable")
    
    return results


# =============================================================================
# VISUALIZATION
# =============================================================================

def create_visualizations(data: pd.DataFrame, output_path: Path, report_log):
    """Generate comprehensive visualizations."""
    
    report_log("\n" + "="*70)
    report_log("VISUALIZATION")
    report_log("="*70)
    
    plt.style.use('seaborn-v0_8-whitegrid')
    
    # Figure 1: Hypothesis results (2x3 grid)
    fig1, axes1 = plt.subplots(2, 3, figsize=(18, 12))
    
    # H1: Attention by Mode
    ax = axes1[0, 0]
    seq_att = data.loc[data['tested_sequence'], 'seq_attention'].dropna()
    ond_att = data.loc[data['tested_ondemand'], 'ond_attention'].dropna()
    plot_data = pd.DataFrame({
        'Mode': ['Sequence']*len(seq_att) + ['On-demand']*len(ond_att),
        'Attention': list(seq_att) + list(ond_att)
    })
    sns.boxplot(x='Mode', y='Attention', data=plot_data, ax=ax, palette='Set2')
    sns.stripplot(x='Mode', y='Attention', data=plot_data, ax=ax, color='black', alpha=0.6, size=10)
    ax.set_title(f'H1: Attention by Mode\n(Seq n={len(seq_att)}, OnD n={len(ond_att)})', fontweight='bold')
    ax.set_ylabel('Attention (1-5)\nHigher = Less cognitive load')
    ax.set_ylim(0.5, 5.5)
    
    # H2: SUS by Preference
    ax = axes1[0, 1]
    sus_by_pref = data[data['pref_category'].isin(['On-demand', 'Sequence'])][['pref_category', 'sus_total']].dropna()
    if len(sus_by_pref) > 0:
        sns.boxplot(x='pref_category', y='sus_total', data=sus_by_pref, ax=ax, palette='Set3')
        sns.stripplot(x='pref_category', y='sus_total', data=sus_by_pref, ax=ax, color='black', alpha=0.6, size=10)
        ax.axhline(y=68, color='red', linestyle='--', alpha=0.7, label='Benchmark (68)')
        ax.axhline(y=85.5, color='green', linestyle=':', alpha=0.7, label='Excellent (85.5)')
        ax.legend()
    ax.set_title('H2: SUS by Preference', fontweight='bold')
    ax.set_ylabel('SUS Score (0-100)')
    
    # H3: Tech Background vs Learnability (ALL participants)
    ax = axes1[0, 2]
    valid_h3 = data[data['tech_composite'].notna() & data['learnability'].notna()]
    colors = ['green' if dof == 1 else 'red' for dof in valid_h3['tech_dof_correct']]
    ax.scatter(valid_h3['tech_composite'], valid_h3['learnability'], s=120, c=colors, 
               edgecolors='black', linewidth=1.5, alpha=0.7)
    for _, row in valid_h3.iterrows():
        ax.annotate(f"P{int(row['participant_id'])}", (row['tech_composite'], row['learnability']),
                   textcoords="offset points", xytext=(5, 5), fontsize=9)
    if len(valid_h3) >= 2:
        z = np.polyfit(valid_h3['tech_composite'], valid_h3['learnability'], 1)
        p = np.poly1d(z)
        x_line = np.linspace(valid_h3['tech_composite'].min(), valid_h3['tech_composite'].max(), 100)
        ax.plot(x_line, p(x_line), 'b--', alpha=0.7, linewidth=2)
        rho, _ = spearmanr(valid_h3['tech_composite'], valid_h3['learnability'])
        ax.text(0.05, 0.95, f'ρ = {rho:.3f}', transform=ax.transAxes, fontsize=12, 
                verticalalignment='top', fontweight='bold')
    ax.scatter([], [], c='green', s=80, label='DoF Correct', edgecolors='black')
    ax.scatter([], [], c='red', s=80, label='DoF Incorrect', edgecolors='black')
    ax.legend(loc='lower right')
    ax.set_title(f'H3: Tech Background vs Learnability (n={len(valid_h3)})', fontweight='bold')
    ax.set_xlabel('Technical Background (1-5)')
    ax.set_ylabel('Learnability (0-100)')
    
    # H4: Control vs On-demand Preference
    ax = axes1[1, 0]
    ond_testers = data[data['tested_ondemand'] & data['ond_control'].notna()]
    if len(ond_testers) > 0:
        colors_h4 = ['blue' if p == 1 else 'orange' for p in ond_testers['prefers_ondemand']]
        ax.scatter(ond_testers['participant_id'], ond_testers['ond_control'], s=150, 
                   c=colors_h4, edgecolors='black', linewidth=1.5)
        for _, row in ond_testers.iterrows():
            ax.annotate(f"P{int(row['participant_id'])}", (row['participant_id'], row['ond_control']),
                       textcoords="offset points", xytext=(5, 5), fontsize=9)
        ax.scatter([], [], c='blue', s=80, label='Prefers On-demand', edgecolors='black')
        ax.scatter([], [], c='orange', s=80, label='Prefers Other', edgecolors='black')
        ax.legend()
    ax.set_title(f'H4: Control Score by Preference (n={len(ond_testers)})', fontweight='bold')
    ax.set_xlabel('Participant')
    ax.set_ylabel('Control Score (1-5)')
    ax.set_ylim(0.5, 5.5)
    
    # H5: Safety vs Sequence Preference
    ax = axes1[1, 1]
    # Compute H5 safety composite
    data_h5 = data.copy()
    data_h5['h5_safety'] = data_h5[['safety_speed', 'safety_awareness']].mean(axis=1)
    seq_testers = data_h5['tested_sequence'] == True
    data_h5.loc[seq_testers, 'h5_safety'] = data_h5.loc[seq_testers, 
        ['safety_speed', 'safety_awareness', 'seq_trust']].mean(axis=1)
    valid_h5 = data_h5[data_h5['h5_safety'].notna()]
    
    if len(valid_h5) > 0:
        colors_h5 = ['purple' if p == 1 else 'gray' for p in valid_h5['prefers_sequence']]
        ax.scatter(valid_h5['participant_id'], valid_h5['h5_safety'], s=150,
                   c=colors_h5, edgecolors='black', linewidth=1.5)
        for _, row in valid_h5.iterrows():
            ax.annotate(f"P{int(row['participant_id'])}", (row['participant_id'], row['h5_safety']),
                       textcoords="offset points", xytext=(5, 5), fontsize=9)
        ax.scatter([], [], c='purple', s=80, label='Prefers Sequence', edgecolors='black')
        ax.scatter([], [], c='gray', s=80, label='Prefers Other', edgecolors='black')
        ax.legend()
    ax.set_title(f'H5: Safety Score by Preference (n={len(valid_h5)})', fontweight='bold')
    ax.set_xlabel('Participant')
    ax.set_ylabel('Safety Composite (1-5)')
    ax.set_ylim(0.5, 5.5)
    
    # SUS Distribution
    ax = axes1[1, 2]
    sus_scores = data['sus_total'].dropna()
    sns.histplot(sus_scores, kde=True, ax=ax, color='steelblue', bins=6, alpha=0.7)
    ax.axvline(x=sus_scores.mean(), color='red', linestyle='-', linewidth=2, label=f'Mean: {sus_scores.mean():.1f}')
    ax.axvline(x=68, color='orange', linestyle='--', linewidth=2, label='Benchmark (68)')
    ax.axvline(x=85.5, color='green', linestyle=':', linewidth=2, label='Excellent (85.5)')
    ax.set_title(f'SUS Distribution (n={len(sus_scores)})', fontweight='bold')
    ax.set_xlabel('SUS Score')
    ax.legend()
    
    plt.tight_layout()
    fig1.savefig(output_path / 'hypothesis_plots.png', dpi=150, bbox_inches='tight')
    plt.close(fig1)
    report_log(f"  Saved: hypothesis_plots.png")
    
    # Figure 2: Score Distributions (2x3 grid)
    fig2, axes2 = plt.subplots(2, 3, figsize=(18, 12))
    
    # Technical Background Distribution
    ax = axes2[0, 0]
    tech_scores = data['tech_composite'].dropna()
    sns.histplot(tech_scores, kde=True, ax=ax, color='teal', bins=6, alpha=0.7)
    ax.axvline(x=tech_scores.mean(), color='red', linestyle='-', linewidth=2, label=f'Mean: {tech_scores.mean():.2f}')
    ax.set_title('Technical Background Distribution', fontweight='bold')
    ax.set_xlabel('Technical Background (1-5)')
    ax.legend()
    
    # Learnability Distribution
    ax = axes2[0, 1]
    learn_scores = data['learnability'].dropna()
    sns.histplot(learn_scores, kde=True, ax=ax, color='coral', bins=6, alpha=0.7)
    ax.axvline(x=learn_scores.mean(), color='red', linestyle='-', linewidth=2, label=f'Mean: {learn_scores.mean():.1f}')
    ax.set_title('Learnability Distribution', fontweight='bold')
    ax.set_xlabel('Learnability (0-100)')
    ax.legend()
    
    # Safety Perception Distribution
    ax = axes2[0, 2]
    safety_scores = data['safety_perception_mean'].dropna()
    sns.histplot(safety_scores, kde=True, ax=ax, color='mediumpurple', bins=6, alpha=0.7)
    ax.axvline(x=safety_scores.mean(), color='red', linestyle='-', linewidth=2, label=f'Mean: {safety_scores.mean():.2f}')
    ax.set_title('Safety Perception Distribution', fontweight='bold')
    ax.set_xlabel('Safety Perception (1-5)')
    ax.legend()
    
    # Control Score Distribution (On-demand testers)
    ax = axes2[1, 0]
    control_scores = data.loc[data['tested_ondemand'], 'ond_control'].dropna()
    if len(control_scores) > 0:
        sns.histplot(control_scores, kde=True, ax=ax, color='dodgerblue', bins=5, alpha=0.7)
        ax.axvline(x=control_scores.mean(), color='red', linestyle='-', linewidth=2, label=f'Mean: {control_scores.mean():.2f}')
    ax.set_title(f'Control Score Distribution (n={len(control_scores)})', fontweight='bold')
    ax.set_xlabel('Control (1-5)')
    ax.legend()
    
    # Exhaustion Distribution (On-demand testers)
    ax = axes2[1, 1]
    exh_scores = data.loc[data['tested_ondemand'], 'ond_exhaustion'].dropna()
    if len(exh_scores) > 0:
        sns.histplot(exh_scores, kde=True, ax=ax, color='salmon', bins=5, alpha=0.7)
        ax.axvline(x=exh_scores.mean(), color='red', linestyle='-', linewidth=2, label=f'Mean: {exh_scores.mean():.2f}')
        ax.axvline(x=3, color='gray', linestyle='--', linewidth=2, label='Neutral (3)')
    ax.set_title(f'Mental Exhaustion Distribution (n={len(exh_scores)})', fontweight='bold')
    ax.set_xlabel('Exhaustion (1-5)')
    ax.legend()
    
    # Godspeed Safety Distribution
    ax = axes2[1, 2]
    godspeed_scores = data['godspeed_safety_mean'].dropna()
    sns.histplot(godspeed_scores, kde=True, ax=ax, color='forestgreen', bins=6, alpha=0.7)
    ax.axvline(x=godspeed_scores.mean(), color='red', linestyle='-', linewidth=2, label=f'Mean: {godspeed_scores.mean():.2f}')
    ax.set_title('Godspeed Safety Distribution', fontweight='bold')
    ax.set_xlabel('Godspeed Safety (1-5)')
    ax.legend()
    
    plt.tight_layout()
    fig2.savefig(output_path / 'distribution_plots.png', dpi=150, bbox_inches='tight')
    plt.close(fig2)
    report_log(f"  Saved: distribution_plots.png")


# =============================================================================
# MAIN PIPELINE
# =============================================================================

def main(input_file: str):
    input_path = Path(input_file)
    output_dir = Path("./hri_results")
    output_dir.mkdir(exist_ok=True)
    
    report = StringIO()
    
    def log(msg):
        print(msg)
        report.write(msg + "\n")
    
    log("\n" + "="*70)
    log("  HRI QUESTIONNAIRE ANALYSIS - Dum-E Robot Study")
    log("="*70)
    log(f"\n  Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    log(f"  Input: {input_file}")
    
    # Extract data
    extractor = HRIDataExtractor(input_file)
    data = extractor.extract_all()
    report.write(extractor.get_report())
    
    # Compute scores
    data = compute_sus_scores(data, log)
    data = compute_learnability_scores(data, log)
    data = add_preference_categories(data)
    
    # Hypothesis testing
    all_results = {}
    all_results['H1'] = analyze_h1(data, log)
    all_results['H2'] = analyze_h2(data, log)
    all_results['H3'] = analyze_h3(data, log)
    all_results['H4'] = analyze_h4(data, log)
    all_results['H5'] = analyze_h5(data, log)
    
    # Reliability
    all_results['reliability'] = analyze_reliability(data, log)
    
    # Visualization
    create_visualizations(data, output_dir, log)
    
    # Final Summary
    log("\n" + "="*70)
    log("FINAL SUMMARY")
    log("="*70)
    
    log(f"\n  SAMPLE:")
    log(f"    Total: {len(data)}")
    log(f"    Tested Sequence: {data['tested_sequence'].sum()}")
    log(f"    Tested On-demand: {data['tested_ondemand'].sum()}")
    log(f"    Tested BOTH: {data['tested_both'].sum()}")
    
    log(f"\n  HYPOTHESIS RESULTS (α = 0.05):")
    log("  " + "-"*50)
    for h in ['H1', 'H2', 'H3', 'H4', 'H5']:
        status = "✓ SUPPORTED" if all_results[h].get('significant') else "✗ NOT SUPPORTED"
        log(f"    {h}: {status}")
    
    log(f"\n  RELIABILITY (Cronbach's α):")
    for scale, result in all_results['reliability'].items():
        log(f"    {scale}: α = {result['alpha']:.3f} ({result['interpretation']})")
    
    log(f"\n  KEY METRICS:")
    log(f"    SUS: {data['sus_total'].mean():.1f} ± {data['sus_total'].std():.1f}")
    log(f"    Tech Background: {data['tech_composite'].mean():.2f} ± {data['tech_composite'].std():.2f}")
    log(f"    Learnability: {data['learnability'].mean():.1f} ± {data['learnability'].std():.1f}")
    
    # Save outputs
    log("\n  OUTPUT FILES:")
    
    report_path = output_dir / 'analysis_report.txt'
    with open(report_path, 'w') as f:
        f.write(report.getvalue())
    log(f"    {report_path}")
    
    data.to_csv(output_dir / 'processed_data.csv', index=False)
    log(f"    {output_dir / 'processed_data.csv'}")
    
    def convert(obj):
        if isinstance(obj, (np.integer, np.floating)):
            return float(obj)
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if isinstance(obj, (np.bool_, bool)):
            return bool(obj)
        return obj
    
    with open(output_dir / 'results.json', 'w') as f:
        json.dump(json.loads(json.dumps(all_results, default=convert)), f, indent=2)
    log(f"    {output_dir / 'results.json'}")
    
    log(f"    {output_dir / 'hypothesis_plots.png'}")
    log(f"    {output_dir / 'distribution_plots.png'}")
    
    log("\n" + "="*70)
    log("  ANALYSIS COMPLETE")
    log("="*70 + "\n")
    
    return all_results


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python hri_analysis.py <excel_file.xlsx>")
        sys.exit(1)
    
    if not Path(sys.argv[1]).exists():
        print(f"Error: File not found: {sys.argv[1]}")
        sys.exit(1)
    
    main(sys.argv[1])
