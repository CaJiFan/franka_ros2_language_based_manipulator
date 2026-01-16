# Dum-E HRI Evaluation: Statistical Analysis Documentation

## Overview

This document provides comprehensive documentation for the Human-Robot Interaction (HRI) evaluation of the Dum-E voice-controlled collaborative robot system. The analysis evaluates two hypotheses comparing Sequential and On-Demand operational modes, along with overall system usability assessment.

**Study Details:**
- **Total Participants:** n = 25
- **Data Source:** `Dum-E_Answers_new.xlsx`
- **Questionnaire:** [Google Forms Link](https://docs.google.com/forms/d/e/1FAIpQLSdJeLDewR_Z2syaXNlzRHvq5y7sgkgfPRB4cPdLWET6-f5qRg/viewform)
- **Analysis Date:** January 2026

---

## Hypotheses

### H1: Attentional Demand

> **"Users have to focus less on the robot and its movements during Sequence mode compared to On-Demand mode, as the user does not have to actively interact with the robot to use it."**

| Aspect | Details |
|--------|---------|
| **Type** | Within-subjects comparison |
| **IV** | Operational Mode (Sequence vs On-Demand) |
| **DV** | Focus score ("I barely had to focus on the robot's individual movements") |
| **Scale** | 1-5 Likert (higher = less attention required) |
| **Prediction** | Focus_Sequence > Focus_On-Demand |

### H2: Learnability vs Technical Background

> **"Users with higher technical background perceive the system as easier to learn."**

| Aspect | Details |
|--------|---------|
| **Type** | Correlation analysis |
| **IV** | Technical Background (composite score) |
| **DV** | Learnability (SUS subscale, 0-100) |
| **Prediction** | Positive correlation (ρ > 0) |

---

## Variable Operationalization

### Technical Background (H2 - Independent Variable)

**Formula:**
```
Technical_Background = 0.6 × Self_Assessment_Mean + 0.4 × DoF_Score × 5
```

**Components:**

| Component | Weight | Source | Description |
|-----------|--------|--------|-------------|
| Self-Assessment | 60% | Columns G-M (7 items) | Mean of robotics familiarity items |
| DoF Score | 40% | Column N | Binary (1=correct, 0=incorrect) × 5 |

**Self-Assessment Items (G-M):**
- G: Understanding of robot motion planning
- H: Comfort working near robotic arms
- I: Understanding of robot sensors
- J: Familiarity with Computer Vision
- K: Familiarity with ROS
- L: Familiarity with Cobots
- M: Familiarity with Teleoperation

**DoF Scoring Criterion:**
- Correct if response contains "independent" AND ("joint" OR "axes")

**Scale Range:** 0.6 (minimum) to 5.0 (maximum)

### Learnability (H2 - Dependent Variable)

**Source:** SUS Learnability Subscale (4 items)

| Item | Column | Description | Polarity |
|------|--------|-------------|----------|
| AT | 45 | "I thought the system was easy to use" | Positive |
| AU | 46 | "I would need technical support" | Negative |
| AX | 49 | "Most people would learn quickly" | Positive |
| AZ | 51 | "I felt confident using the system" | Positive |

**Scoring:**
1. Positive items: score = raw - 1 (range 0-4)
2. Negative items: score = 5 - raw (range 0-4)
3. Learnability = mean(scores) × 25 → 0-100 scale

### Focus Scores (H1)

| Mode | Column | Condition |
|------|--------|-----------|
| Sequence | 32 (AG) | Both-mode testers |
| On-Demand | 36 (AK) | Both-mode testers |
| Sequence | 23 (X) | Single-mode (Seq only) |
| On-Demand | 27 (AB) | Single-mode (OnD only) |

---

## Sample Description

### Testing Conditions

| Condition | n | Used For |
|-----------|---|----------|
| **Both modes tested** | 20 | Paired H1 comparison |
| → On-Demand first, then Sequence | 11 | — |
| → Sequence first, then On-Demand | 9 | — |
| **Sequence only** | 4 | Excluded from paired test |
| **On-Demand only** | 1 | Excluded from paired test |
| **Total** | 25 | H2 correlation |

### Descriptive Statistics

#### H1 Variables

| Variable | n | Mean | SD | Median | Range |
|----------|---|------|-------|--------|-------|
| Focus (Sequence) | 24 | 4.00 | 0.93 | 4.0 | [1, 5] |
| Focus (On-Demand) | 21 | 3.19 | 1.44 | 3.0 | [1, 5] |
| Paired Difference | 20 | +0.90 | — | — | [-1, 4] |

#### H2 Variables

| Variable | n | Mean | SD | Median | Range |
|----------|---|------|-------|--------|-------|
| Technical Background | 25 | 3.68 | 1.03 | 3.97 | [1.03, 5.00] |
| Learnability | 25 | 80.0 | 14.0 | 81.25 | [50, 100] |

#### Additional Variables

| Variable | n | Mean | SD | Median |
|----------|---|------|-------|--------|
| Predictability | 25 | 4.56 | 0.58 | 5.0 |
| DoF Correct | 25 | 88% (22/25) | — | — |

---

## Statistical Analysis Pipeline

### General Approach

```
Step 1: Normality Test (Shapiro-Wilk)
              ↓
Step 2: Test Selection (parametric vs non-parametric)
              ↓
Step 3: Hypothesis Test (compute statistic & p-value)
              ↓
Step 4: Effect Size Interpretation
```

### H1 Analysis Pipeline

#### Step 1: Normality Test

| Test | W | p-value | Decision |
|------|---|---------|----------|
| Shapiro-Wilk (paired differences) | 0.9004 | 0.0420 | NOT normal |

#### Step 2: Test Selection

- **Design:** Paired (within-subjects)
- **Normality:** Violated (p < 0.05)
- **Selected Test:** Wilcoxon signed-rank test

#### Step 3: Hypothesis Test

| Statistic | Value |
|-----------|-------|
| Wilcoxon W | 6.0 |
| p-value | **0.0086** |
| n (pairs) | 20 |

**Decision:** Reject H₀ (p < 0.05)

#### Step 4: Effect Size

| Metric | Value | Interpretation |
|--------|-------|----------------|
| Rank-biserial r | 0.943 | **LARGE** |
| Mean difference | +0.90 | Sequence > On-Demand |

**Within-Subject Consistency:**
- Seq > OnD (expected): 10/20 (50%)
- Seq = OnD (tied): 8/20 (40%)
- Seq < OnD (opposite): 2/20 (10%)
- **Consistency:** 90% in expected or neutral direction

### H2 Analysis Pipeline

#### Step 1: Normality Test

| Variable | W | p-value | Decision |
|----------|---|---------|----------|
| Technical Background | 0.8989 | 0.0174 | NOT normal |
| Learnability | 0.9183 | 0.0469 | NOT normal |

#### Step 2: Test Selection

- **Design:** Correlation
- **Normality:** Both violated
- **Selected Test:** Spearman's rank correlation

#### Step 3: Correlation Analysis

| Statistic | Value |
|-----------|-------|
| Spearman's ρ | **0.597** |
| p-value | **0.0016** |
| n | 25 |

**Decision:** Reject H₀ (p < 0.05)

#### Step 4: Effect Size

| Metric | Value | Interpretation |
|--------|-------|----------------|
| Spearman's ρ | 0.597 | **MODERATE** positive |
| R² | 0.356 | 35.6% variance explained |

---

## Results Summary

### Hypothesis Testing

| Hypothesis | Test | Statistic | p-value | Effect Size | Result |
|------------|------|-----------|---------|-------------|--------|
| **H1** | Wilcoxon | W = 6.0 | 0.0086 | r = 0.943 (large) | **SUPPORTED ✓** |
| **H2** | Spearman | ρ = 0.597 | 0.0016 | R² = 35.6% (moderate) | **SUPPORTED ✓** |

### System Usability Scale (SUS)

| Metric | Value |
|--------|-------|
| Mean | **77.9** |
| SD | 11.8 |
| Median | 80.0 |
| 95% CI | [72.8, 83.0] |
| Grade | **B (Good)** |
| Adjective | "Good" |

---

## Interpretation

### H1: Attentional Demand

**Finding:** Users report requiring significantly **less attentional focus** during Sequence mode compared to On-Demand mode (W = 6.0, p = 0.0086, r = 0.943).

**Interpretation:** The autonomous operation in Sequence mode reduces cognitive load by eliminating the need for active voice commands for each object. The large effect size (r = 0.943) indicates this is a practically meaningful difference.

### H2: Learnability vs Technical Background

**Finding:** There is a **moderate positive correlation** between technical background and perceived learnability (ρ = 0.597, p = 0.0016).

**Interpretation:** Users with greater robotics familiarity perceive the system as easier to learn. However, with R² = 35.6%, the majority of variance (64.4%) is explained by other factors, suggesting the system remains accessible to users with varying expertise levels.

### Overall Usability

**Finding:** SUS score of **77.9** (Grade B, "Good").

**Interpretation:** The system achieves good usability, exceeding the industry average of 68. The voice-controlled interface provides an intuitive interaction paradigm suitable for both technical and non-technical users.

---

## Output Files

| File | Description |
|------|-------------|
| `h1_h2_analysis.py` | Complete analysis script |
| `Hypothesis_Analysis_Report.txt` | Full statistical report |
| `hypothesis_plots.png` | H1 bar chart + H2 scatter plot |
| `distribution_plots.png` | 6-panel variable distributions |
| `testing_conditions.png` | Testing condition bar chart |
| `processed_data.csv` | Extracted participant data |
| `results.json` | Machine-readable results |
| `section7_user_study.tex` | LaTeX section for technical report |

---

## Usage

### Running the Analysis

```bash
# Basic usage (file in current directory)
python hri_analysis.py Dum-E_Answers_new.xlsx

# Or with explicit path
python hri_analysis.py /path/to/Dum-E_Answers_new.xlsx
```

### Requirements

```
pandas
numpy
scipy
matplotlib
openpyxl
```

### Output Directory

Results are saved to `./hypothesis_output/`

---

## Statistical Methods Reference

### Effect Size Conventions (Cohen)

| Correlation | Interpretation |
|-------------|----------------|
| |r| < 0.20 | Negligible |
| |r| 0.20-0.39 | Weak |
| |r| 0.40-0.69 | Moderate |
| |r| ≥ 0.70 | Strong |

### SUS Score Interpretation (Bangor et al., 2009)

| Score Range | Grade | Adjective |
|-------------|-------|-----------|
| 85-100 | A | Excellent |
| 70-84 | B | Good |
| 50-69 | C | OK |
| < 50 | D/F | Poor |

---

## Limitations

1. **Sample Size:** n = 25 limits detection of small effects
2. **Self-Report Bias:** Technical background partially relies on self-assessment
3. **Single Objective Measure:** Only one DoF question validates technical knowledge
4. **Order Effects:** Potential learning/fatigue effects not fully controlled
5. **Laboratory Setting:** Results may not generalize to real-world deployment

---

## Citation

If using this analysis, please cite:

```
Dum-E: Voice-guided Collaborative Robot for Handover in Sequential Tasks
CIR-MAI Course Project
Universitat Politècnica de Catalunya (UPC)
January 2026
```

---

