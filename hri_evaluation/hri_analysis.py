#!/usr/bin/env python3
"""
================================================================================
HYPOTHESIS ANALYSIS: H1 (Attentional Demand) & H2 (Learnability vs Tech Background)
Dum-E Robot Interaction Study
================================================================================
"""

import pandas as pd
import numpy as np
from scipy import stats
from scipy.stats import spearmanr, pearsonr, shapiro, wilcoxon, mannwhitneyu
import matplotlib.pyplot as plt
from pathlib import Path
import json
from datetime import datetime

# =============================================================================
# CONFIGURATION
# =============================================================================

# H1 Columns
COL_TESTING_MODE = 22
COL_PREDICTABILITY = 21  # V: "I found the robot's movements predictable and easy to follow"

# Focus columns (attention) - structure based on testing order
COL_FOCUS_SEQ_FIRST = 23   # X: Sequence focus for single-mode or embedded in combined
COL_FOCUS_OND_FIRST = 27   # AB: On-demand focus (single tester P24)
COL_FOCUS_SEQ_BOTH = 32    # AG: Sequence focus for both-mode testers
COL_FOCUS_OND_BOTH = 36    # AK: On-demand focus for both-mode testers

# H2 Columns
COL_SELF_ASSESS = [6, 7, 8, 9, 10, 11, 12]  # G-M
COL_DOF_QUESTION = 13  # N
COL_LEARN_AT = 45
COL_LEARN_AU = 46
COL_LEARN_AX = 49
COL_LEARN_AZ = 51

WEIGHT_SELF_ASSESS = 0.6
WEIGHT_DOF = 0.4


# =============================================================================
# UTILITY FUNCTIONS
# =============================================================================

def parse_cell_value(value):
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


def check_dof_correct(answer):
    if pd.isna(answer):
        return 0
    answer_lower = str(answer).lower()
    if "independent" in answer_lower and ("joint" in answer_lower or "axes" in answer_lower):
        return 1
    return 0


# =============================================================================
# DATA EXTRACTION
# =============================================================================

def extract_all_data(filepath):
    """Extract data for both H1 and H2."""
    
    df = pd.read_excel(filepath)
    n = len(df)
    
    results = []
    
    for idx in range(n):
        row = df.iloc[idx]
        
        # --- Testing Mode ---
        testing_mode = str(row.iloc[COL_TESTING_MODE])
        
        # Determine testing condition
        if 'First On Demand' in testing_mode:
            condition = 'OnD_then_Seq'
            tested_both = True
        elif 'First Sequence' in testing_mode:
            condition = 'Seq_then_OnD'
            tested_both = True
        elif testing_mode == 'Sequence Mode':
            condition = 'Seq_only'
            tested_both = False
        elif testing_mode == 'On Demand Mode':
            condition = 'OnD_only'
            tested_both = False
        else:
            condition = 'Unknown'
            tested_both = False
        
        # --- H1: Attentional Demand ---
        
        # Predictability (common to all)
        predictability = parse_cell_value(row.iloc[COL_PREDICTABILITY])
        
        # Focus scores depend on testing condition
        if condition == 'Seq_only':
            focus_seq = parse_cell_value(row.iloc[COL_FOCUS_SEQ_FIRST])
            focus_ond = np.nan
        elif condition == 'OnD_only':
            focus_seq = np.nan
            focus_ond = parse_cell_value(row.iloc[COL_FOCUS_OND_FIRST])
        else:  # Both modes
            focus_seq = parse_cell_value(row.iloc[COL_FOCUS_SEQ_BOTH])
            focus_ond = parse_cell_value(row.iloc[COL_FOCUS_OND_BOTH])
        
        # --- H2: Technical Background ---
        self_assess_items = [parse_cell_value(row.iloc[c]) for c in COL_SELF_ASSESS]
        self_assess_mean = np.nanmean(self_assess_items)
        dof_correct = check_dof_correct(row.iloc[COL_DOF_QUESTION])
        tech_composite = WEIGHT_SELF_ASSESS * self_assess_mean + WEIGHT_DOF * dof_correct * 5
        
        # --- H2: Learnability ---
        raw_at = parse_cell_value(row.iloc[COL_LEARN_AT])
        raw_au = parse_cell_value(row.iloc[COL_LEARN_AU])
        raw_ax = parse_cell_value(row.iloc[COL_LEARN_AX])
        raw_az = parse_cell_value(row.iloc[COL_LEARN_AZ])
        
        score_at = raw_at - 1 if pd.notna(raw_at) else np.nan
        score_au = 5 - raw_au if pd.notna(raw_au) else np.nan
        score_ax = raw_ax - 1 if pd.notna(raw_ax) else np.nan
        score_az = raw_az - 1 if pd.notna(raw_az) else np.nan
        
        scores = [score_at, score_au, score_ax, score_az]
        valid_scores = [s for s in scores if pd.notna(s)]
        learnability = np.mean(valid_scores) * 25 if len(valid_scores) > 0 else np.nan
        
        results.append({
            'participant_id': idx,
            'testing_mode': testing_mode,
            'condition': condition,
            'tested_both': tested_both,
            # H1
            'focus_seq': focus_seq,
            'focus_ond': focus_ond,
            'predictability': predictability,
            # H2
            'self_assess_mean': self_assess_mean,
            'dof_correct': dof_correct,
            'tech_composite': tech_composite,
            'learnability': learnability,
        })
    
    return pd.DataFrame(results)


# =============================================================================
# H1 ANALYSIS
# =============================================================================

def analyze_h1(data, report):
    """Analyze H1: Attentional Demand - Sequence vs On-Demand."""
    
    def log(msg):
        print(msg)
        report.append(msg)
    
    log("\n" + "=" * 80)
    log("HYPOTHESIS 1: ATTENTIONAL DEMAND")
    log("=" * 80)
    
    # -------------------------------------------------------------------------
    # 1. VARIABLE OPERATIONALIZATION
    # -------------------------------------------------------------------------
    
    log("\n" + "-" * 80)
    log("1. VARIABLE OPERATIONALIZATION")
    log("-" * 80)
    
    log("""
    HYPOTHESIS H1:
        "Users have to focus less on the robot during Sequence mode 
         compared to On-Demand mode."
    
    PREDICTION:
        Focus_Sequence > Focus_On-Demand
        (Higher score = LESS focus required, per item wording)
    
    VARIABLES:
    
    A) Attentional Demand - Sequence Mode
       Item: "I barely had to focus on the robot's individual movements 
              while executing the task" (Sequence version)
       Scale: 1-5 Likert (1 = Strongly Disagree, 5 = Strongly Agree)
       Interpretation: Higher score = Less attentional demand (good)
    
    B) Attentional Demand - On-Demand Mode  
       Item: Same wording (On-Demand version)
       Scale: 1-5 Likert
       Interpretation: Higher score = Less attentional demand (good)
    
    C) Predictability (Supporting Variable)
       Item: "I found the robot's movements predictable and easy to follow"
       Scale: 1-5 Likert
       Used for: Correlation analysis (do predictable movements reduce attention?)
    
    DESIGN:
        Within-subjects comparison for participants who tested both modes.
        Participants who tested only one mode are excluded from paired analysis.
    """)
    
    # -------------------------------------------------------------------------
    # 2. DATA EXTRACTION
    # -------------------------------------------------------------------------
    
    log("\n" + "-" * 80)
    log("2. DATA EXTRACTION")
    log("-" * 80)
    
    log("\n  ID  | Condition        | Focus_Seq | Focus_OnD | Predictability")
    log("  " + "-" * 65)
    
    for _, row in data.iterrows():
        cond = row['condition'][:15]
        f_seq = f"{row['focus_seq']:.0f}" if pd.notna(row['focus_seq']) else "NA"
        f_ond = f"{row['focus_ond']:.0f}" if pd.notna(row['focus_ond']) else "NA"
        pred = f"{row['predictability']:.0f}" if pd.notna(row['predictability']) else "NA"
        log(f"  P{int(row['participant_id']):2} | {cond:16} | {f_seq:9} | {f_ond:9} | {pred}")
    
    # -------------------------------------------------------------------------
    # 3. SAMPLE DESCRIPTION
    # -------------------------------------------------------------------------
    
    log("\n" + "-" * 80)
    log("3. SAMPLE DESCRIPTION")
    log("-" * 80)
    
    both_mode = data[data['tested_both'] == True]
    seq_only = data[data['condition'] == 'Seq_only']
    ond_only = data[data['condition'] == 'OnD_only']
    
    log(f"""
    TESTING CONDITIONS:
    
    Total Participants: n = {len(data)}
    
    - Tested BOTH modes:     n = {len(both_mode)} (used for paired comparison)
      - On-Demand first:     n = {len(data[data['condition'] == 'OnD_then_Seq'])}
      - Sequence first:      n = {len(data[data['condition'] == 'Seq_then_OnD'])}
    
    - Tested Sequence ONLY:  n = {len(seq_only)} (excluded from paired test)
    - Tested On-Demand ONLY: n = {len(ond_only)} (excluded from paired test)
    """)
    
    # -------------------------------------------------------------------------
    # 4. DESCRIPTIVE STATISTICS
    # -------------------------------------------------------------------------
    
    log("\n" + "-" * 80)
    log("4. DESCRIPTIVE STATISTICS")
    log("-" * 80)
    
    # All available Sequence scores
    all_seq = data['focus_seq'].dropna()
    # All available On-Demand scores  
    all_ond = data['focus_ond'].dropna()
    # Paired data only
    paired = both_mode[['focus_seq', 'focus_ond']].dropna()
    
    log(f"""
    FOCUS SCORES (all available data):
    
    Sequence Mode (n = {len(all_seq)}):
        Mean   = {all_seq.mean():.3f}
        SD     = {all_seq.std():.3f}
        Median = {all_seq.median():.1f}
        Range  = [{all_seq.min():.0f}, {all_seq.max():.0f}]
    
    On-Demand Mode (n = {len(all_ond)}):
        Mean   = {all_ond.mean():.3f}
        SD     = {all_ond.std():.3f}
        Median = {all_ond.median():.1f}
        Range  = [{all_ond.min():.0f}, {all_ond.max():.0f}]
    
    PAIRED DATA (both-mode testers, n = {len(paired)}):
        Sequence Mean   = {paired['focus_seq'].mean():.3f}
        On-Demand Mean  = {paired['focus_ond'].mean():.3f}
        Mean Difference = {paired['focus_seq'].mean() - paired['focus_ond'].mean():.3f}
    """)
    
    # Predictability
    pred = data['predictability'].dropna()
    log(f"""
    PREDICTABILITY (n = {len(pred)}):
        Mean   = {pred.mean():.3f}
        SD     = {pred.std():.3f}
        Median = {pred.median():.1f}
    """)
    
    # -------------------------------------------------------------------------
    # 5. STATISTICAL ANALYSIS PIPELINE
    # -------------------------------------------------------------------------
    
    log("\n" + "-" * 80)
    log("5. STATISTICAL ANALYSIS PIPELINE")
    log("-" * 80)
    
    log("""
    PIPELINE OVERVIEW:
    
        Step 5.1: Test normality of paired differences (Shapiro-Wilk)
                           ↓
        Step 5.2: Select appropriate paired test
                           ↓
        Step 5.3: Compute test statistic and significance
                           ↓
        Step 5.4: Effect size and practical interpretation
                           ↓
        Step 5.5: Supporting analysis (Predictability correlation)
    """)
    
    # --- STEP 5.1: NORMALITY TEST ---
    
    log("\n" + "-" * 40)
    log("STEP 5.1: NORMALITY TEST (Shapiro-Wilk)")
    log("-" * 40)
    
    differences = paired['focus_seq'] - paired['focus_ond']
    
    log(f"""
    For paired comparisons, we test normality of the DIFFERENCES.
    
    Paired Differences (Sequence - On-Demand):
        n = {len(differences)}
        Values: {[f'{d:.0f}' for d in differences.values]}
    """)
    
    if len(differences) >= 3:
        w_diff, p_diff = shapiro(differences)
        log(f"""
    Shapiro-Wilk Test:
        H₀: Differences are normally distributed
        H₁: Differences are NOT normally distributed
        
        W statistic = {w_diff:.4f}
        p-value     = {p_diff:.4f}
        
        Decision: {"REJECT H₀ (NOT normal)" if p_diff < 0.05 else "FAIL TO REJECT H₀ (appears normal)"}
    """)
        use_parametric = p_diff >= 0.05
    else:
        log("    Insufficient data for normality test.")
        use_parametric = False
        p_diff = np.nan
    
    # --- STEP 5.2: TEST SELECTION ---
    
    log("\n" + "-" * 40)
    log("STEP 5.2: TEST SELECTION")
    log("-" * 40)
    
    log(f"""
    DESIGN: Paired (within-subjects) comparison
    
    Test Options:
        - Parametric: Paired t-test (requires normal differences)
        - Non-parametric: Wilcoxon signed-rank test (no normality assumption)
    
    Decision Rationale:
        {"Differences appear normally distributed → Paired t-test acceptable" if use_parametric else "Normality violated OR small sample → Wilcoxon signed-rank test"}
    
    SELECTED TEST: {"PAIRED T-TEST" if use_parametric else "WILCOXON SIGNED-RANK TEST"}
    """)
    
    # --- STEP 5.3: HYPOTHESIS TEST ---
    
    log("\n" + "-" * 40)
    log("STEP 5.3: HYPOTHESIS TEST")
    log("-" * 40)
    
    log(f"""
    HYPOTHESES:
        H₀: Median(Focus_Seq) = Median(Focus_OnD)  [No difference]
        H₁: Median(Focus_Seq) ≠ Median(Focus_OnD)  [Difference exists]
    
    DIRECTIONAL PREDICTION:
        Focus_Seq > Focus_OnD (Sequence requires less attention)
    
    SIGNIFICANCE LEVEL: α = 0.05 (two-tailed)
    """)
    
    seq_vals = paired['focus_seq'].values
    ond_vals = paired['focus_ond'].values
    
    # Wilcoxon signed-rank test
    try:
        w_stat, p_wilcox = wilcoxon(seq_vals, ond_vals)
        
        log(f"""
    WILCOXON SIGNED-RANK TEST:
    
        Test statistic W = {w_stat:.1f}
        p-value          = {p_wilcox:.4f}
        n (pairs)        = {len(paired)}
    """)
        
        significant = p_wilcox < 0.05
        
        log(f"""
    COMPARISON:
        p {"<" if significant else "≥"} α
        {p_wilcox:.4f} {"<" if significant else "≥"} 0.05
    
    DECISION: {"REJECT H₀" if significant else "FAIL TO REJECT H₀"}
    
    INTERPRETATION:
        {"The difference IS statistically significant." if significant else "The difference is NOT statistically significant."}
    """)
        
    except Exception as e:
        log(f"    Error in Wilcoxon test: {e}")
        w_stat = np.nan
        p_wilcox = np.nan
        significant = False
    
    # --- STEP 5.4: EFFECT SIZE ---
    
    log("\n" + "-" * 40)
    log("STEP 5.4: EFFECT SIZE")
    log("-" * 40)
    
    # Rank-biserial correlation (effect size for Wilcoxon)
    n_pairs = len(paired)
    r_effect = 1 - (2 * w_stat) / (n_pairs * (n_pairs + 1) / 2) if n_pairs > 0 else np.nan
    
    # Direction check
    mean_diff = paired['focus_seq'].mean() - paired['focus_ond'].mean()
    direction_correct = mean_diff > 0
    
    # Effect size interpretation
    abs_r = abs(r_effect) if pd.notna(r_effect) else 0
    if abs_r >= 0.5:
        strength = "LARGE"
    elif abs_r >= 0.3:
        strength = "MEDIUM"
    elif abs_r >= 0.1:
        strength = "SMALL"
    else:
        strength = "NEGLIGIBLE"
    
    log(f"""
    EFFECT SIZE (Rank-biserial correlation):
    
        r = {r_effect:.4f}
        
        Cohen's conventions for r:
            |r| < 0.10  →  Negligible
            |r| 0.10-0.29  →  Small
            |r| 0.30-0.49  →  Medium  
            |r| ≥ 0.50  →  Large
        
        Observed |r| = {abs_r:.3f} → {strength}
    
    DIRECTION CHECK:
        Mean Difference (Seq - OnD) = {mean_diff:.3f}
        Prediction: Sequence > On-Demand (positive difference)
        Observed:   {"CONSISTENT with prediction ✓" if direction_correct else "OPPOSITE to prediction ✗"}
    
    WITHIN-SUBJECT CONSISTENCY:
    """)
    
    # Count direction of differences
    pos_diff = sum(differences > 0)
    zero_diff = sum(differences == 0)
    neg_diff = sum(differences < 0)
    
    log(f"""        Seq > OnD (expected):  {pos_diff}/{n_pairs} ({pos_diff/n_pairs*100:.0f}%)
        Seq = OnD (tied):      {zero_diff}/{n_pairs} ({zero_diff/n_pairs*100:.0f}%)
        Seq < OnD (opposite):  {neg_diff}/{n_pairs} ({neg_diff/n_pairs*100:.0f}%)
        
        Consistency: {(pos_diff + zero_diff)/n_pairs*100:.0f}% in expected or neutral direction
    """)
    
    # --- STEP 5.5: SUPPORTING ANALYSIS ---
    
    log("\n" + "-" * 40)
    log("STEP 5.5: SUPPORTING ANALYSIS (Predictability Correlation)")
    log("-" * 40)
    
    log("""
    RATIONALE:
        If predictable movements reduce attentional demand, then higher
        predictability scores should correlate with higher focus scores
        (i.e., less attention required).
    """)
    
    # Correlation: Predictability with Focus (using all available data)
    # Combine focus scores for correlation
    combined_focus = []
    combined_pred = []
    
    for _, row in data.iterrows():
        pred_val = row['predictability']
        if pd.notna(pred_val):
            if pd.notna(row['focus_seq']):
                combined_focus.append(row['focus_seq'])
                combined_pred.append(pred_val)
            if pd.notna(row['focus_ond']):
                combined_focus.append(row['focus_ond'])
                combined_pred.append(pred_val)
    
    if len(combined_focus) >= 5:
        rho_pred, p_pred = spearmanr(combined_pred, combined_focus)
        
        log(f"""
    Spearman Correlation: Predictability × Focus Score
    
        n (observations) = {len(combined_focus)}
        Spearman's ρ     = {rho_pred:.4f}
        p-value          = {p_pred:.4f}
        
        Interpretation:
            {"Significant positive correlation: Predictability IS associated with lower attention." if (p_pred < 0.05 and rho_pred > 0) else "No significant correlation found."}
    """)
    else:
        rho_pred, p_pred = np.nan, np.nan
        log("    Insufficient data for correlation analysis.")
    
    # -------------------------------------------------------------------------
    # 6. HYPOTHESIS CONCLUSION
    # -------------------------------------------------------------------------
    
    log("\n" + "-" * 80)
    log("6. HYPOTHESIS CONCLUSION")
    log("-" * 80)
    
    h1_supported = significant and direction_correct
    
    log(f"""
    HYPOTHESIS H1:
        "Users have to focus less on the robot during Sequence mode
         compared to On-Demand mode."
    
    STATISTICAL EVIDENCE:
        - Wilcoxon W        = {w_stat:.1f}
        - p-value           = {p_wilcox:.4f}
        - Effect size (r)   = {r_effect:.3f} ({strength})
        - Mean difference   = {mean_diff:.3f}
        - Direction correct = {direction_correct}
    
    RESULT: {"H1 SUPPORTED ✓" if h1_supported else "H1 NOT SUPPORTED ✗"}
    """)
    
    if h1_supported:
        log(f"""
    CONCLUSION:
        There is a statistically significant difference in attentional demand
        between modes (W = {w_stat:.1f}, p = {p_wilcox:.4f}).
        
        Users report requiring LESS focus during Sequence mode compared to
        On-Demand mode, with a {strength.lower()} effect size (r = {r_effect:.3f}).
        
        This supports the hypothesis that autonomous sequence execution
        reduces cognitive load compared to active voice-command interaction.
    """)
    else:
        log(f"""
    CONCLUSION:
        {"The difference is not statistically significant (p = " + f"{p_wilcox:.4f}" + " > 0.05)." if not significant else "The difference is in the OPPOSITE direction to the prediction."}
    """)
    
    # Return results for plotting
    return {
        'significant': significant,
        'w_stat': w_stat,
        'p_value': p_wilcox,
        'effect_size': r_effect,
        'effect_strength': strength,
        'mean_diff': mean_diff,
        'direction_correct': direction_correct,
        'h1_supported': h1_supported,
        'paired_data': paired,
        'n_pairs': n_pairs,
        'seq_mean': paired['focus_seq'].mean(),
        'ond_mean': paired['focus_ond'].mean(),
        'seq_sd': paired['focus_seq'].std(),
        'ond_sd': paired['focus_ond'].std(),
        'rho_predictability': rho_pred,
        'p_predictability': p_pred
    }


# =============================================================================
# H2 ANALYSIS
# =============================================================================

def analyze_h2(data, report):
    """Analyze H2: Learnability vs Technical Background."""
    
    def log(msg):
        print(msg)
        report.append(msg)
    
    log("\n" + "=" * 80)
    log("HYPOTHESIS 2: LEARNABILITY VS TECHNICAL BACKGROUND")
    log("=" * 80)
    
    # -------------------------------------------------------------------------
    # 1. VARIABLE OPERATIONALIZATION
    # -------------------------------------------------------------------------
    
    log("\n" + "-" * 80)
    log("1. VARIABLE OPERATIONALIZATION")
    log("-" * 80)
    
    log("""
    HYPOTHESIS H2:
        "Users with higher technical background perceive the system 
         as easier to learn."
    
    INDEPENDENT VARIABLE: Technical Background
    
        Formula: Tech = 0.6 × Self_Assessment_Mean + 0.4 × DoF_Score × 5
        
        Components:
        A) Self-Assessment Mean (60% weight)
           - 7 items (columns G-M): Understanding of robotics concepts
           - Scale: 1-5 Likert each
           
        B) DoF Score (40% weight)
           - Objective test: "What does Degrees of Freedom refer to?"
           - Binary: 1 if correct (mentions "independent joints/axes"), 0 otherwise
           - Scaled ×5 to match 1-5 range
        
        Scale Range: 0.6 (minimum) to 5.0 (maximum)
    
    DEPENDENT VARIABLE: Learnability
    
        Source: SUS Learnability Subscale (4 items)
        
        Items:
        - AT: "Easy to use" (positive)
        - AU: "Need technical support" (negative → reversed)
        - AX: "Learn quickly" (positive)
        - AZ: "Confident" (positive)
        
        Scoring: Transform to 0-4, average, multiply by 25 → 0-100 scale
    """)
    
    # -------------------------------------------------------------------------
    # 2. DATA EXTRACTION
    # -------------------------------------------------------------------------
    
    log("\n" + "-" * 80)
    log("2. DATA EXTRACTION")
    log("-" * 80)
    
    log("\n  ID  | Self-Assess | DoF | Tech Composite | Learnability")
    log("  " + "-" * 60)
    
    for _, row in data.iterrows():
        dof_sym = '✓' if row['dof_correct'] == 1 else '✗'
        log(f"  P{int(row['participant_id']):2} |    {row['self_assess_mean']:.2f}     |  {dof_sym}  |      {row['tech_composite']:.2f}       |    {row['learnability']:.2f}")
    
    # -------------------------------------------------------------------------
    # 3. DESCRIPTIVE STATISTICS
    # -------------------------------------------------------------------------
    
    log("\n" + "-" * 80)
    log("3. DESCRIPTIVE STATISTICS")
    log("-" * 80)
    
    tech = data['tech_composite'].dropna()
    learn = data['learnability'].dropna()
    
    log(f"""
    TECHNICAL BACKGROUND (n = {len(tech)}):
        Mean   = {tech.mean():.3f}
        SD     = {tech.std():.3f}
        Median = {tech.median():.3f}
        Range  = [{tech.min():.2f}, {tech.max():.2f}]
        
        DoF Question: {int(data['dof_correct'].sum())}/{len(data)} correct ({data['dof_correct'].sum()/len(data)*100:.1f}%)
    
    LEARNABILITY (n = {len(learn)}):
        Mean   = {learn.mean():.3f}
        SD     = {learn.std():.3f}
        Median = {learn.median():.3f}
        Range  = [{learn.min():.2f}, {learn.max():.2f}]
    """)
    
    # -------------------------------------------------------------------------
    # 4. STATISTICAL ANALYSIS PIPELINE
    # -------------------------------------------------------------------------
    
    log("\n" + "-" * 80)
    log("4. STATISTICAL ANALYSIS PIPELINE")
    log("-" * 80)
    
    log("""
    PIPELINE:
        Step 4.1: Test normality (Shapiro-Wilk)
                           ↓
        Step 4.2: Select correlation test
                           ↓
        Step 4.3: Compute correlation and significance
                           ↓
        Step 4.4: Effect size interpretation
    """)
    
    # --- STEP 4.1: NORMALITY ---
    
    log("\n" + "-" * 40)
    log("STEP 4.1: NORMALITY TEST (Shapiro-Wilk)")
    log("-" * 40)
    
    w_tech, p_tech = shapiro(tech)
    w_learn, p_learn = shapiro(learn)
    
    log(f"""
    Technical Background:
        W = {w_tech:.4f}, p = {p_tech:.4f}
        Decision: {"NOT normal" if p_tech < 0.05 else "Appears normal"}
    
    Learnability:
        W = {w_learn:.4f}, p = {p_learn:.4f}
        Decision: {"NOT normal" if p_learn < 0.05 else "Appears normal"}
    """)
    
    use_spearman = (p_tech < 0.05) or (p_learn < 0.05)
    
    log(f"""
    CONCLUSION:
        {"At least one variable violates normality." if use_spearman else "Both variables appear normal."}
        SELECTED TEST: {"SPEARMAN'S RANK CORRELATION" if use_spearman else "PEARSON'S CORRELATION"}
    """)
    
    # --- STEP 4.2: CORRELATION ---
    
    log("\n" + "-" * 40)
    log("STEP 4.2: CORRELATION ANALYSIS")
    log("-" * 40)
    
    valid_mask = data['tech_composite'].notna() & data['learnability'].notna()
    valid_data = data[valid_mask]
    n_valid = len(valid_data)
    
    tech_vals = valid_data['tech_composite'].values
    learn_vals = valid_data['learnability'].values
    
    rho, p_spearman = spearmanr(tech_vals, learn_vals)
    r_pearson, p_pearson = pearsonr(tech_vals, learn_vals)
    
    if use_spearman:
        main_stat, main_p, main_name = rho, p_spearman, "Spearman's ρ"
    else:
        main_stat, main_p, main_name = r_pearson, p_pearson, "Pearson's r"
    
    log(f"""
    HYPOTHESES:
        H₀: ρ = 0 (no correlation)
        H₁: ρ ≠ 0 (correlation exists)
    
    RESULTS:
        {main_name} = {main_stat:.4f}
        p-value     = {main_p:.4f}
        n           = {n_valid}
    """)
    
    significant = main_p < 0.05
    
    log(f"""
    DECISION:
        p {"<" if significant else "≥"} α (0.05)
        {"REJECT H₀" if significant else "FAIL TO REJECT H₀"}
        
        The correlation {"IS" if significant else "is NOT"} statistically significant.
    """)
    
    # --- STEP 4.3: EFFECT SIZE ---
    
    log("\n" + "-" * 40)
    log("STEP 4.3: EFFECT SIZE")
    log("-" * 40)
    
    abs_stat = abs(main_stat)
    if abs_stat >= 0.7:
        strength = "STRONG"
    elif abs_stat >= 0.4:
        strength = "MODERATE"
    elif abs_stat >= 0.2:
        strength = "WEAK"
    else:
        strength = "NEGLIGIBLE"
    
    direction = "POSITIVE" if main_stat > 0 else "NEGATIVE"
    r_squared = main_stat ** 2
    
    log(f"""
    Effect Size: {strength} {direction}
        |{main_name}| = {abs_stat:.3f}
        
        Cohen's conventions:
        |r| < 0.20 → Negligible
        |r| 0.20-0.39 → Weak
        |r| 0.40-0.69 → Moderate
        |r| ≥ 0.70 → Strong
    
    Variance Explained: R² = {r_squared:.4f} ({r_squared*100:.1f}%)
    """)
    
    # -------------------------------------------------------------------------
    # 5. HYPOTHESIS CONCLUSION
    # -------------------------------------------------------------------------
    
    log("\n" + "-" * 80)
    log("5. HYPOTHESIS CONCLUSION")
    log("-" * 80)
    
    h2_supported = significant and (main_stat > 0)
    
    log(f"""
    HYPOTHESIS H2:
        "Users with higher technical background perceive the system
         as easier to learn."
    
    STATISTICAL EVIDENCE:
        - {main_name}  = {main_stat:.3f}
        - p-value      = {main_p:.4f}
        - Effect size  = {strength} {direction}
        - R²           = {r_squared:.1%}
    
    RESULT: {"H2 SUPPORTED ✓" if h2_supported else "H2 NOT SUPPORTED ✗"}
    """)
    
    if h2_supported:
        log(f"""
    CONCLUSION:
        There is a statistically significant {strength.lower()} positive correlation
        between technical background and perceived learnability
        ({main_name} = {main_stat:.3f}, p = {main_p:.4f}).
        
        Users with greater technical expertise rate the system as easier to learn.
        Technical background explains {r_squared*100:.1f}% of learnability variance.
    """)
    
    return {
        'significant': significant,
        'correlation': main_stat,
        'p_value': main_p,
        'test_name': main_name,
        'effect_strength': strength,
        'r_squared': r_squared,
        'h2_supported': h2_supported,
        'tech_vals': tech_vals,
        'learn_vals': learn_vals,
        'n': n_valid,
        'tech_mean': tech.mean(),
        'tech_sd': tech.std(),
        'learn_mean': learn.mean(),
        'learn_sd': learn.std()
    }


# =============================================================================
# VISUALIZATION
# =============================================================================

def create_visualizations(data, h1_results, h2_results, output_dir):
    """Create comprehensive visualization plots for 25 participants."""
    
    # --- FIGURE 1: HYPOTHESIS TESTS ---
    fig1, axes1 = plt.subplots(1, 2, figsize=(14, 6))
    
    # H1 PLOT: Bar chart with individual points
    ax1 = axes1[0]
    
    paired = h1_results['paired_data']
    categories = ['Sequence Mode', 'On-Demand Mode']
    means = [h1_results['seq_mean'], h1_results['ond_mean']]
    sds = [h1_results['seq_sd'], h1_results['ond_sd']]
    
    bars = ax1.bar(categories, means, yerr=sds, capsize=8, 
                   color=['steelblue', 'coral'], edgecolor='black', linewidth=1.5,
                   alpha=0.8)
    
    ax1.set_ylabel('Focus Score (1-5)\n(Higher = Less Attention Required)', fontsize=11, fontweight='bold')
    ax1.set_ylim(0, 5.5)
    
    # Add individual paired data points with connecting lines
    for _, row in paired.iterrows():
        ax1.plot([0, 1], [row['focus_seq'], row['focus_ond']], 
                 'o-', color='gray', alpha=0.4, markersize=6, linewidth=1)
    
    # Add statistics box
    result_str = "SUPPORTED" if h1_results['h1_supported'] else "NOT SUPPORTED"
    stats_text = (f"n = {h1_results['n_pairs']} pairs\n"
                  f"W = {h1_results['w_stat']:.1f}\n"
                  f"p = {h1_results['p_value']:.4f}\n"
                  f"r = {h1_results['effect_size']:.3f}")
    ax1.text(0.02, 0.98, stats_text, transform=ax1.transAxes,
             fontsize=10, verticalalignment='top',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    ax1.set_title(f'H1: Attentional Demand by Mode\nResult: {result_str}', 
                  fontsize=13, fontweight='bold')
    
    # H2 PLOT: Scatter plot
    ax2 = axes1[1]
    
    ax2.scatter(h2_results['tech_vals'], h2_results['learn_vals'], 
               s=100, c='steelblue', alpha=0.7, edgecolors='black', linewidth=1.5)
    
    # Regression line
    z = np.polyfit(h2_results['tech_vals'], h2_results['learn_vals'], 1)
    p_line = np.poly1d(z)
    x_line = np.linspace(min(h2_results['tech_vals']) - 0.3, 
                         max(h2_results['tech_vals']) + 0.3, 100)
    ax2.plot(x_line, p_line(x_line), 'r--', linewidth=2, alpha=0.8)
    
    ax2.set_xlabel('Technical Background (Composite)', fontsize=11, fontweight='bold')
    ax2.set_ylabel('Learnability (0-100)', fontsize=11, fontweight='bold')
    
    result_str = "SUPPORTED" if h2_results['h2_supported'] else "NOT SUPPORTED"
    stats_text = (f"n = {h2_results['n']}\n"
                  f"{h2_results['test_name']} = {h2_results['correlation']:.3f}\n"
                  f"p = {h2_results['p_value']:.4f}\n"
                  f"R² = {h2_results['r_squared']:.3f}")
    ax2.text(0.02, 0.98, stats_text, transform=ax2.transAxes,
             fontsize=10, verticalalignment='top',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    ax2.set_title(f'H2: Technical Background vs Learnability\nResult: {result_str}',
                  fontsize=13, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    fig1.savefig(output_dir / 'hypothesis_plots.png', dpi=150, bbox_inches='tight')
    plt.close(fig1)
    
    # --- FIGURE 2: DATA DISTRIBUTIONS ---
    fig2, axes2 = plt.subplots(2, 3, figsize=(15, 10))
    
    # Plot 2.1: Focus Sequence Distribution (all n=24)
    ax = axes2[0, 0]
    seq_all = data['focus_seq'].dropna()
    ax.hist(seq_all, bins=[0.5, 1.5, 2.5, 3.5, 4.5, 5.5], 
            color='steelblue', edgecolor='black', alpha=0.7, rwidth=0.8)
    ax.axvline(seq_all.mean(), color='red', linestyle='-', linewidth=2, label=f'Mean: {seq_all.mean():.2f}')
    ax.axvline(seq_all.median(), color='orange', linestyle='--', linewidth=2, label=f'Median: {seq_all.median():.1f}')
    ax.set_xlabel('Focus Score (Sequence Mode)', fontsize=10)
    ax.set_ylabel('Frequency', fontsize=10)
    ax.set_title(f'Focus - Sequence Mode\n(n = {len(seq_all)})', fontweight='bold')
    ax.set_xticks([1, 2, 3, 4, 5])
    ax.legend(fontsize=9)
    ax.set_xlim(0.5, 5.5)
    
    # Plot 2.2: Focus On-Demand Distribution (all n=21)
    ax = axes2[0, 1]
    ond_all = data['focus_ond'].dropna()
    ax.hist(ond_all, bins=[0.5, 1.5, 2.5, 3.5, 4.5, 5.5], 
            color='coral', edgecolor='black', alpha=0.7, rwidth=0.8)
    ax.axvline(ond_all.mean(), color='red', linestyle='-', linewidth=2, label=f'Mean: {ond_all.mean():.2f}')
    ax.axvline(ond_all.median(), color='orange', linestyle='--', linewidth=2, label=f'Median: {ond_all.median():.1f}')
    ax.set_xlabel('Focus Score (On-Demand Mode)', fontsize=10)
    ax.set_ylabel('Frequency', fontsize=10)
    ax.set_title(f'Focus - On-Demand Mode\n(n = {len(ond_all)})', fontweight='bold')
    ax.set_xticks([1, 2, 3, 4, 5])
    ax.legend(fontsize=9)
    ax.set_xlim(0.5, 5.5)
    
    # Plot 2.3: Paired Differences Distribution
    ax = axes2[0, 2]
    differences = h1_results['paired_data']['focus_seq'] - h1_results['paired_data']['focus_ond']
    bins_diff = np.arange(differences.min() - 0.5, differences.max() + 1.5, 1)
    ax.hist(differences, bins=bins_diff, color='purple', edgecolor='black', alpha=0.7, rwidth=0.8)
    ax.axvline(0, color='black', linestyle=':', linewidth=2, label='No difference')
    ax.axvline(differences.mean(), color='red', linestyle='-', linewidth=2, label=f'Mean: {differences.mean():.2f}')
    ax.set_xlabel('Difference (Seq - OnD)', fontsize=10)
    ax.set_ylabel('Frequency', fontsize=10)
    ax.set_title(f'Paired Differences\n(n = {len(differences)} pairs)', fontweight='bold')
    ax.legend(fontsize=9)
    
    # Plot 2.4: Technical Background Distribution (n=25)
    ax = axes2[1, 0]
    tech = data['tech_composite'].dropna()
    ax.hist(tech, bins=8, color='teal', edgecolor='black', alpha=0.7)
    ax.axvline(tech.mean(), color='red', linestyle='-', linewidth=2, label=f'Mean: {tech.mean():.2f}')
    ax.axvline(tech.median(), color='orange', linestyle='--', linewidth=2, label=f'Median: {tech.median():.2f}')
    ax.set_xlabel('Technical Background (Composite)', fontsize=10)
    ax.set_ylabel('Frequency', fontsize=10)
    ax.set_title(f'Technical Background Distribution\n(n = {len(tech)})', fontweight='bold')
    ax.legend(fontsize=9)
    
    # Plot 2.5: Learnability Distribution (n=25)
    ax = axes2[1, 1]
    learn = data['learnability'].dropna()
    ax.hist(learn, bins=8, color='gold', edgecolor='black', alpha=0.7)
    ax.axvline(learn.mean(), color='red', linestyle='-', linewidth=2, label=f'Mean: {learn.mean():.1f}')
    ax.axvline(learn.median(), color='orange', linestyle='--', linewidth=2, label=f'Median: {learn.median():.1f}')
    ax.set_xlabel('Learnability Score (0-100)', fontsize=10)
    ax.set_ylabel('Frequency', fontsize=10)
    ax.set_title(f'Learnability Distribution\n(n = {len(learn)})', fontweight='bold')
    ax.legend(fontsize=9)
    
    # Plot 2.6: Predictability Distribution (n=25)
    ax = axes2[1, 2]
    pred = data['predictability'].dropna()
    ax.hist(pred, bins=[0.5, 1.5, 2.5, 3.5, 4.5, 5.5], 
            color='lightgreen', edgecolor='black', alpha=0.7, rwidth=0.8)
    ax.axvline(pred.mean(), color='red', linestyle='-', linewidth=2, label=f'Mean: {pred.mean():.2f}')
    ax.axvline(pred.median(), color='orange', linestyle='--', linewidth=2, label=f'Median: {pred.median():.1f}')
    ax.set_xlabel('Predictability Score', fontsize=10)
    ax.set_ylabel('Frequency', fontsize=10)
    ax.set_title(f'Predictability Distribution\n(n = {len(pred)})', fontweight='bold')
    ax.set_xticks([1, 2, 3, 4, 5])
    ax.legend(fontsize=9)
    ax.set_xlim(0.5, 5.5)
    
    plt.tight_layout()
    fig2.savefig(output_dir / 'distribution_plots.png', dpi=150, bbox_inches='tight')
    plt.close(fig2)
    
    # --- FIGURE 3: TESTING CONDITIONS BAR CHART ---
    fig3, ax3 = plt.subplots(figsize=(10, 6))
    
    conditions = ['Sequence Only', 'On-Demand First,\nthen Sequence', 
                  'Sequence First,\nthen On-Demand', 'On-Demand Only']
    counts = [
        len(data[data['condition'] == 'Seq_only']),
        len(data[data['condition'] == 'OnD_then_Seq']),
        len(data[data['condition'] == 'Seq_then_OnD']),
        len(data[data['condition'] == 'OnD_only'])
    ]
    colors = ['steelblue', 'mediumpurple', 'mediumseagreen', 'coral']
    
    bars = ax3.bar(conditions, counts, color=colors, edgecolor='black', linewidth=1.5)
    
    # Add count labels on bars
    for bar, count in zip(bars, counts):
        ax3.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.3,
                str(count), ha='center', va='bottom', fontsize=12, fontweight='bold')
    
    ax3.set_ylabel('Number of Participants', fontsize=12, fontweight='bold')
    ax3.set_xlabel('Testing Condition', fontsize=12, fontweight='bold')
    ax3.set_title(f'Testing Conditions (Total n = {len(data)})', fontsize=14, fontweight='bold')
    ax3.set_ylim(0, max(counts) + 2)
    
    # Add annotation
    both_mode = counts[1] + counts[2]
    ax3.text(0.98, 0.95, f'Both modes: {both_mode}\nSingle mode: {counts[0] + counts[3]}',
             transform=ax3.transAxes, fontsize=11, verticalalignment='top',
             horizontalalignment='right',
             bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))
    
    plt.tight_layout()
    fig3.savefig(output_dir / 'testing_conditions.png', dpi=150, bbox_inches='tight')
    plt.close(fig3)


# =============================================================================
# MAIN
# =============================================================================

def main():
    import sys
    
    # Accept filepath from command line argument, or use default
    if len(sys.argv) > 1:
        filepath = sys.argv[1]
    else:
        filepath = 'Dum-E_Answers_new.xlsx'  # Default: look in current directory
    
    # Verify file exists
    if not Path(filepath).exists():
        print(f"Error: File not found: {filepath}")
        print(f"Usage: python {sys.argv[0]} <excel_file.xlsx>")
        sys.exit(1)
    
    output_dir = Path('./hypothesis_output')
    output_dir.mkdir(exist_ok=True)
    
    print("Extracting data...")
    data = extract_all_data(filepath)
    
    report = []
    report.append("=" * 80)
    report.append("DUM-E ROBOT INTERACTION STUDY: HYPOTHESIS ANALYSIS")
    report.append("=" * 80)
    report.append(f"\nAnalysis Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    report.append(f"Total Participants: n = {len(data)}")
    
    print("\nAnalyzing H1...")
    h1_results = analyze_h1(data, report)
    
    print("\nAnalyzing H2...")
    h2_results = analyze_h2(data, report)
    
    # Summary
    report.append("\n" + "=" * 80)
    report.append("SUMMARY OF RESULTS")
    report.append("=" * 80)
    report.append(f"""
    H1 (Attentional Demand): {"SUPPORTED ✓" if h1_results['h1_supported'] else "NOT SUPPORTED ✗"}
        Sequence mode requires less attention than On-Demand mode.
        W = {h1_results['w_stat']:.1f}, p = {h1_results['p_value']:.4f}, r = {h1_results['effect_size']:.3f}
    
    H2 (Learnability vs Technical Background): {"SUPPORTED ✓" if h2_results['h2_supported'] else "NOT SUPPORTED ✗"}
        Technical background correlates with perceived learnability.
        {h2_results['test_name']} = {h2_results['correlation']:.3f}, p = {h2_results['p_value']:.4f}
    """)
    
    # Create visualizations
    print("\nCreating visualizations...")
    create_visualizations(data, h1_results, h2_results, output_dir)
    
    # Save report
    report_path = output_dir / 'Hypothesis_Analysis_Report.txt'
    with open(report_path, 'w') as f:
        f.write('\n'.join(report))
    
    # Save data
    data.to_csv(output_dir / 'processed_data.csv', index=False)
    
    # Save results JSON
    results_json = {
        'H1': {
            'supported': bool(h1_results['h1_supported']),
            'test': 'Wilcoxon signed-rank',
            'W': float(h1_results['w_stat']),
            'p': float(h1_results['p_value']),
            'effect_size_r': float(h1_results['effect_size']),
            'effect_strength': str(h1_results['effect_strength']),
            'n_pairs': int(h1_results['n_pairs']),
            'seq_mean': float(h1_results['seq_mean']),
            'ond_mean': float(h1_results['ond_mean'])
        },
        'H2': {
            'supported': bool(h2_results['h2_supported']),
            'test': str(h2_results['test_name']),
            'correlation': float(h2_results['correlation']),
            'p': float(h2_results['p_value']),
            'r_squared': float(h2_results['r_squared']),
            'effect_strength': str(h2_results['effect_strength']),
            'n': int(h2_results['n'])
        }
    }
    
    with open(output_dir / 'results.json', 'w') as f:
        json.dump(results_json, f, indent=2)
    
    print(f"\nAnalysis complete! Files saved to {output_dir}")
    print("\nOutput files:")
    print("  - Hypothesis_Analysis_Report.txt")
    print("  - hypothesis_plots.png")
    print("  - distribution_plots.png")
    print("  - testing_conditions.png")
    print("  - processed_data.csv")
    print("  - results.json")


if __name__ == "__main__":
    main()
