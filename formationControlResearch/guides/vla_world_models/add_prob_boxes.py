"""Inserts the green probability/derivation boxes at the first place each idea is used.
Each entry: (file, anchor text, box text). The box is inserted right AFTER the anchor.
Fails loudly if an anchor is missing or not unique, so nothing is inserted twice."""
import sys

B = []  # (file, anchor, text)

# ---------------------------------------------------------------- main file: legend entry
B.append(("vla_wm_notes.tex", r"""A short summary of one paper: what problem, what idea, what result. The full list with arXiv numbers is in the Sources section at the end.
\end{paperbox}""", r"""
\begin{probbox}{name of the idea}
Explains a probability or statistics idea the first time it is used, from zero, and derives the formula that appears in the text. Later chapters point back to these boxes instead of repeating them.
\end{probbox}"""))

# ---------------------------------------------------------------- P1 sigmoid
B.append(("part1.tex", r"""Without $\phi$ the whole stack would collapse into one linear map $W_L\cdots W_1\x$, which is just linear regression again. The non-linearity is what lets the network bend its decision boundary.""", r"""

\begin{probbox}{random variables, Bernoulli, and why the sigmoid gives a probability}
\textbf{Random variable.} A quantity whose value is uncertain, written with a capital letter, $Y$. Its \emph{distribution} says how likely each value is. A binary label $Y\in\{0,1\}$ (``cat or not'') has the \textbf{Bernoulli} distribution with one parameter $p=P(Y=1)$:
\[
P(Y=y)=p^{\,y}(1-p)^{1-y}\qquad(y=1\text{ gives }p,\quad y=0\text{ gives }1-p).
\]
\textbf{Conditional probability.} $P(A\mid B)=P(A\cap B)/P(B)$ is the probability of $A$ once we know that $B$ happened. A classifier models $P(Y=1\mid\x)$: the probability of the label \emph{given} the input.

\textbf{Odds and log-odds.} The odds are $p/(1-p)$: $p=0.8$ gives odds $4$ (``4 to 1''). The log-odds $\log\frac{p}{1-p}$ can be any real number, from $-\infty$ ($p\to0$) to $+\infty$ ($p\to1$), so it is a natural thing for a linear model to predict.

\textbf{Derivation of the sigmoid.} Logistic regression \emph{assumes} that the log-odds is linear in $\x$, $\log\frac{p}{1-p}=z=\w^\top\x+b$. Solve for $p$:
\[
\frac{p}{1-p}=e^{z}\ \Rightarrow\ p=e^{z}-p\,e^{z}\ \Rightarrow\ p\,(1+e^{z})=e^{z}\ \Rightarrow\ p=\frac{e^{z}}{1+e^{z}}=\frac{1}{1+e^{-z}}=\sigma(z).
\]
So the sigmoid is not an arbitrary squashing function: it is the inverse of the log-odds. Example: $z=\log 4\approx1.386$ gives $\sigma(z)=0.8$.
\end{probbox}"""))

# ---------------------------------------------------------------- P2 likelihood
B.append(("part1.tex", r"""\item If $y$ is one of $K$ classes and the model outputs probabilities $p_1,\dots,p_K$, $-\log p$ is $-\log p_{y}$: the \textbf{cross-entropy} from classification.
\end{itemize}""", r"""

\begin{probbox}{likelihood, independence, and where MSE and cross-entropy come from}
\textbf{Independence.} Two events are independent if knowing one does not change the other: $P(A\cap B)=P(A)P(B)$. We assume the training examples are \emph{independent and identically distributed} (i.i.d.): drawn separately from the same distribution. Then the probability of the whole dataset is a product:
\[
P(y_1,\dots,y_N\mid\x_1,\dots,\x_N)=\prod_{n=1}^N p_\theta(y_n\mid\x_n)\ =:\ \mathcal{L}ik(\theta).
\]
Read as a function of the parameters $\theta$, this product is the \textbf{likelihood}. \textbf{Maximum likelihood} picks the $\theta$ that makes the observed data most probable.

\textbf{Why the log.} $\log$ is increasing, so maximising $\log\mathcal Lik$ gives the same $\theta$. It turns the product into a sum, $\log\prod_n p_n=\sum_n\log p_n$, which is easy to differentiate, and it avoids numerical underflow (a product of a million numbers below 1 is 0 in floating point). Negating and dividing by $N$ gives equation~\eqref{eq:nll}: minimise the average negative log-likelihood.

\textbf{Densities.} For a continuous $y$, $p(y)$ is a \emph{density}: probabilities are areas under it, $P(a\le Y\le b)=\int_a^b p(y)\,\mathrm dy$, and $p(y)$ itself may exceed 1.

\textbf{Derivation 1: Gaussian $\Rightarrow$ MSE.} The Gaussian (normal) density with mean $\hat y$ and variance $\sigma^2$ is
\[
\mathcal N(y;\hat y,\sigma^2)=\frac{1}{\sqrt{2\pi\sigma^2}}\exp\!\Big(-\frac{(y-\hat y)^2}{2\sigma^2}\Big),
\qquad
-\log\mathcal N=\frac{(y-\hat y)^2}{2\sigma^2}+\tfrac12\log(2\pi\sigma^2).
\]
With $\sigma$ fixed, the second term is a constant and the first is the squared error times $1/(2\sigma^2)$. Same minimiser as MSE.

\textbf{Derivation 2: Bernoulli $\Rightarrow$ binary cross-entropy.} $-\log\big[p^{\,y}(1-p)^{1-y}\big]=-y\log p-(1-y)\log(1-p)$.

\textbf{Derivation 3: categorical $\Rightarrow$ cross-entropy.} With $K$ classes, $P(Y=k)=p_k$. Writing the label as a one-hot vector $\vect t$, $-\log p_y=-\sum_k t_k\log p_k$, because only the true class has $t_k=1$.
\end{probbox}"""))

# ---------------------------------------------------------------- P3 expectation, SGD
B.append(("part1.tex", r"""\item \textbf{Regularisation:} weight decay is the L2 penalty from the course; dropout randomly switches off units during training.
\end{itemize}""", r"""

\begin{probbox}{expectation, the law of large numbers, and why mini-batches work}
\textbf{Expectation.} The expected value is the probability-weighted average: $\E[f(X)]=\sum_x P(x)f(x)$ for discrete $X$, or $\int p(x)f(x)\,\mathrm dx$ for continuous $X$. It is \emph{linear}: $\E[aX+bY]=a\E[X]+b\E[Y]$, always, even if $X$ and $Y$ are dependent. The \textbf{variance} $\mathrm{Var}(X)=\E[(X-\E X)^2]=\E[X^2]-(\E X)^2$ measures spread; $\mathrm{Var}(cX)=c^2\mathrm{Var}(X)$, and for \emph{independent} $X,Y$, $\mathrm{Var}(X+Y)=\mathrm{Var}(X)+\mathrm{Var}(Y)$.

\textbf{Law of large numbers.} The average of many i.i.d.\ samples approaches the expectation. That is why the training loss $\frac1N\sum_n\ell_n$ is a stand-in for the expected loss on new data.

\textbf{Mini-batch gradients are unbiased.} Let the full gradient be $\vect g=\frac1N\sum_{n=1}^N\nabla\ell_n$. Draw one index $i$ uniformly at random. Then
\[
\E[\nabla\ell_i]=\sum_{n=1}^N\tfrac1N\nabla\ell_n=\vect g .
\]
A batch of $B$ such draws averages $B$ unbiased estimates, so it is unbiased too, and (by the variance rules above) its variance is $\mathrm{Var}(\nabla\ell_i)/B$. Bigger batches give less noisy steps, but the noise falls only like $1/\sqrt B$ in standard deviation: 4 times the batch for half the noise.
\end{probbox}"""))

# ---------------------------------------------------------------- P4 KL, reparameterisation
B.append(("part1.tex", r"""The first world model (Ha \& Schmidhuber, 2018) used exactly a VAE as its ``eyes''.""", r"""

\begin{probbox}{KL divergence, the Gaussian KL, and sampling with gradients}
\textbf{KL divergence} measures how different a distribution $q$ is from a distribution $p$:
\[
\mathrm{KL}(q\,\|\,p)=\E_{z\sim q}\Big[\log\frac{q(z)}{p(z)}\Big]=\int q(z)\log\frac{q(z)}{p(z)}\,\mathrm dz .
\]
It is $0$ only when $q=p$, otherwise positive, and it is not symmetric ($\mathrm{KL}(q\|p)\ne\mathrm{KL}(p\|q)$). \emph{Why it is never negative:} $\log$ is concave, so Jensen's inequality says $\E[\log X]\le\log\E[X]$. Hence $-\mathrm{KL}=\E_q[\log\tfrac pq]\le\log\E_q[\tfrac pq]=\log\int q\,\tfrac pq=\log\int p=\log1=0$.

\textbf{Derivation: KL between a Gaussian and the standard Gaussian} (what the VAE loss uses). Let $q=\mathcal N(\mu,s^2)$ and $p=\mathcal N(0,1)$. From the density formula,
\[
\log q(z)-\log p(z)=-\tfrac12\log s^2-\frac{(z-\mu)^2}{2s^2}+\frac{z^2}{2}.
\]
Take the expectation under $q$, using $\E_q[(z-\mu)^2]=s^2$ and $\E_q[z^2]=\mathrm{Var}+\text{mean}^2=s^2+\mu^2$:
\[
\mathrm{KL}(q\|p)=-\tfrac12\log s^2-\tfrac12+\tfrac12(s^2+\mu^2)=\tfrac12\big(\mu^2+s^2-1-\log s^2\big).
\]
It is zero exactly when $\mu=0$ and $s=1$. For a $d$-dimensional latent with independent coordinates, add the $d$ terms.

\textbf{The reparameterisation trick.} We cannot backpropagate through ``draw a random sample''. Instead write $z=\mu+s\,\epsilon$ with $\epsilon\sim\mathcal N(0,1)$. By linearity, $\E[z]=\mu$ and $\mathrm{Var}(z)=s^2\mathrm{Var}(\epsilon)=s^2$, so $z$ has the right distribution; and now $z$ is a differentiable function of $\mu$ and $s$, with the randomness moved into $\epsilon$. Section~\ref{sec:dreamer} derives the full VAE loss (the ELBO).
\end{probbox}"""))

# ---------------------------------------------------------------- P5 softmax
B.append(("part1.tex", r"""Its features are so good at geometry that DINO-WM (Section~\ref{sec:latentplan}) builds a world model directly on top of them.""", r"""

\begin{probbox}{the softmax is a categorical distribution, and its gradient}
\textbf{Definition.} For scores (logits) $s_1,\dots,s_K$, $\ \softmax_k(\vect s)=\dfrac{e^{s_k}}{\sum_j e^{s_j}}$. Every entry is positive and they sum to 1, so the result is a valid \textbf{categorical distribution}: the probability of class $k$.

\textbf{Two classes give the sigmoid.} $\dfrac{e^{s_1}}{e^{s_1}+e^{s_2}}=\dfrac{1}{1+e^{-(s_1-s_2)}}=\sigma(s_1-s_2)$ (divide top and bottom by $e^{s_1}$). Softmax is the multi-class logistic regression.

\textbf{Shift invariance.} Adding the same $c$ to every score multiplies top and bottom by $e^{c}$, which cancels. Libraries subtract $\max_j s_j$ before exponentiating so that $e^{s}$ never overflows.

\textbf{Derivation of the gradient.} The cross-entropy for true class $y$ is $-\log\softmax_y=-s_y+\log\sum_j e^{s_j}$. Differentiate with respect to $s_k$:
\[
\frac{\partial}{\partial s_k}\Big(-s_y+\log\sum_j e^{s_j}\Big)=-\mathbb 1[k=y]+\frac{e^{s_k}}{\sum_j e^{s_j}}=p_k-\mathbb 1[k=y].
\]
``Predicted probability minus target'': it pushes the true class up and every other class down in proportion to how much probability it wrongly took. This one line is the training signal of CLIP, of every LLM and of every token-based VLA.

\textbf{Temperature.} Dividing the scores by $\tau$ (or $T$) before the softmax: as $\tau\to0$ the largest score takes all the probability (a one-hot vector); as $\tau\to\infty$ all classes become equally likely.
\end{probbox}"""))

# ---------------------------------------------------------------- P6 sqrt dk
B.append(("part1.tex", r"""otherwise the softmax saturates and its gradients vanish.""", r"""

\begin{probbox}{why divide by $\sqrt{d_k}$: the variance of a dot product}
Assume the components $q_m$ and $k_m$ of a query and a key are independent, with mean 0 and variance 1 (roughly true at initialisation). The score is $\vect q^\top\vect k=\sum_{m=1}^{d_k}q_mk_m$.
\begin{itemize}[leftmargin=*]
\item One term: $\E[q_mk_m]=\E[q_m]\E[k_m]=0$ (for independent variables the expectation of a product is the product of expectations), and $\mathrm{Var}(q_mk_m)=\E[q_m^2k_m^2]-0=\E[q_m^2]\E[k_m^2]=1\cdot1=1$.
\item The sum of $d_k$ independent terms: variances add, so $\mathrm{Var}(\vect q^\top\vect k)=d_k$ and the standard deviation is $\sqrt{d_k}$.
\item Dividing by $\sqrt{d_k}$: $\mathrm{Var}\big(\vect q^\top\vect k/\sqrt{d_k}\big)=d_k/d_k=1$, whatever the dimension.
\end{itemize}
With $d_k=64$ and no scaling, typical scores would be about $\pm8$; $e^{8}\approx3000$ times $e^{0}$, so the softmax would put almost all weight on one token and its gradient (box above: $p_k-\mathbb 1[k=y]$-type terms with $p$ near 0 or 1) would be almost zero.
\end{probbox}"""))

# ---------------------------------------------------------------- P7 chain rule
B.append(("part1.tex", r"""\label{fig:temp}
\end{figure}""", r"""

\begin{probbox}{the chain rule of probability: why next-token prediction models whole texts}
From the definition of conditional probability, $P(a,b)=P(a)\,P(b\mid a)$. Apply it again and again:
\[
p(x_1,x_2,\dots,x_n)=p(x_1)\,p(x_2\mid x_1)\,p(x_3\mid x_1,x_2)\cdots p(x_n\mid x_{<n})=\prod_{t=1}^n p(x_t\mid x_{<t}).
\]
This is exact; no assumption is made. Taking logs, $\log p(x_{1:n})=\sum_t\log p(x_t\mid x_{<t})$, so the next-token cross-entropy summed over a sequence \emph{is} the negative log-likelihood of the whole sequence. Generating one token at a time, each sampled from its conditional, produces exact samples of the joint distribution (``ancestral sampling''). The 7 action tokens of RT-2 or OpenVLA are generated the same way: $p(a^{(1)},\dots,a^{(7)})=\prod_i p(a^{(i)}\mid a^{(<i)},\text{obs})$.

\textbf{Check Figure~\ref{fig:temp} by hand.} At $T=1$: $e^{3}=20.09$, $e^{2.2}=9.03$, $e^{1}=2.72$, $e^{0.3}=1.35$, $e^{-0.5}=0.61$, sum $33.80$, so $p(\text{``cube''})=20.09/33.80=0.594$.
\end{probbox}"""))

# ---------------------------------------------------------------- P8 mean minimiser, mixture
B.append(("part1.tex", r"""Three families appear in VLAs: discretise the action into tokens and use a softmax (which can represent several modes, Section~\ref{sec:actrep}); diffusion; and flow matching.""", r"""

\begin{probbox}{the best guess under squared error is the mean; mixture distributions}
\textbf{Derivation.} Which single number $c$ minimises the expected squared error $f(c)=\E[(Y-c)^2]$? Expand: $f(c)=\E[Y^2]-2c\,\E[Y]+c^2$. Set the derivative to zero: $-2\E[Y]+2c=0$, so $c=\E[Y]$. With an input, the same argument for each $\x$ separately gives the best predictor $\hat y(\x)=\E[Y\mid\x]$, the \emph{conditional mean}. A network trained with MSE and enough data learns this conditional mean.

\textbf{Bias and variance again.} Adding and subtracting $\E Y$: $\E[(Y-c)^2]=\mathrm{Var}(Y)+(\E Y-c)^2$ (the cross term has expectation $2(\E Y-c)\E[Y-\E Y]=0$). The first part is noise no predictor can remove, the second is the error of the guess. This is the decomposition from your course book, for a single point.

\textbf{Why this is bad here.} Half the mass at $-1$ and half at $+1$ give $\E[Y]=\tfrac12(-1)+\tfrac12(+1)=0$: the pole. (The data in Figure~\ref{fig:bimodal} give \bimean.) Under absolute error the best guess is the \emph{median}, which for this data is also between the modes, so changing the loss does not help; changing the model does.

\textbf{Mixture distributions.} A Gaussian mixture is $p(a)=\sum_{k=1}^K\pi_k\,\mathcal N(a;\mu_k,\sigma_k^2)$ with weights $\pi_k\ge0$, $\sum_k\pi_k=1$. To sample: pick component $k$ with probability $\pi_k$, then sample from that Gaussian. The fit in Figure~\ref{fig:bimodal} found $\pi\approx(0.47,0.53)$, $\mu\approx(-1.03,\,0.98)$, $\sigma\approx0.17$. It was fitted with the EM algorithm, which alternates between computing, for each point, the probability that it came from each component, and refitting each component's mean and spread with those probabilities as weights.
\end{probbox}"""))

# ---------------------------------------------------------------- P9 diffusion gaussians
B.append(("part1.tex", r"""\label{fig:diff}
\end{figure}""", r"""

\begin{probbox}{scaling and adding Gaussians; the curves of Figure~\ref{fig:diff}}
\textbf{Rule.} If $X\sim\mathcal N(m,v)$ and $\epsilon\sim\mathcal N(0,1)$ are independent, then $aX+b\epsilon\sim\mathcal N(am,\ a^2v+b^2)$. The mean follows from linearity of expectation, the variance from $\mathrm{Var}(aX)=a^2\mathrm{Var}(X)$ plus ``variances of independent variables add''; that the result is again Gaussian is a special property of the Gaussian.

\textbf{Why the square roots.} With $a=\sqrt{\bar\alpha_t}$ and $b=\sqrt{1-\bar\alpha_t}$: if the data has variance 1, then $\mathrm{Var}(\x_t)=\bar\alpha_t\cdot1+(1-\bar\alpha_t)=1$ at every step. The noise replaces the signal without changing the overall scale (``variance preserving'').

\textbf{The curves.} Our data is the mixture $\tfrac12\mathcal N(-1,s_0^2)+\tfrac12\mathcal N(+1,s_0^2)$ with $s_0=0.15$. Applying the rule to each component:
\[
p(x_t)=\tfrac12\,\mathcal N\big(x;-\sqrt{\bar\alpha},\ \bar\alpha s_0^2+1-\bar\alpha\big)+\tfrac12\,\mathcal N\big(x;+\sqrt{\bar\alpha},\ \bar\alpha s_0^2+1-\bar\alpha\big).
\]
At $\bar\alpha=1$ this is the data; at $\bar\alpha=0$ both components become $\mathcal N(0,1)$. These are exactly the four curves plotted.

\textbf{Why predicting the noise is enough.} Given $\x_t$ and a noise estimate $\hat{\bm\epsilon}$, invert the forward formula: $\hat\x_0=(\x_t-\sqrt{1-\bar\alpha_t}\,\hat{\bm\epsilon})/\sqrt{\bar\alpha_t}$. By the previous box, the network trained with squared error learns $\E[\bm\epsilon\mid\x_t]$. When $\x_t$ is only slightly noisy it already reveals which mode it came from, so this average is taken within one mode, and the modes are not averaged together.
\end{probbox}"""))

# ---------------------------------------------------------------- P10 flow matching cond. expectation
B.append(("part1.tex", r"""\label{fig:flow}
\end{figure}""", r"""

\begin{probbox}{why regressing on single pairs gives the right velocity field}
Many grey lines in Figure~\ref{fig:flow} pass through the same point $(\tau,x)$, each with its own target velocity $u=x_1-x_0$. The network cannot match all of them. By the ``best guess is the mean'' box, the minimiser of $\E\lVert\vect v(\x_\tau,\tau)-\vect u\rVert^2$ is the conditional mean
\[
\vect v^\star(\x,\tau)=\E\big[\x_1-\x_0\ \big|\ \x_\tau=\x\big],
\]
the average velocity of all lines through that point. Lipman et al.\ (2023) proved that following this averaged field from $\tau=0$ to $1$ carries the noise distribution exactly onto the data distribution, even though no single sample moves along a straight line any more. Sampling uses Euler steps, $\x_{\tau+\Delta}=\x_\tau+\Delta\,\vect v_\theta(\x_\tau,\tau)$; with $\Delta=0.1$ that is ten network calls.
\end{probbox}"""))

# ---------------------------------------------------------------- P11 Markov, geometric series
B.append(("part2.tex", r"""``Markov'' means $s_t$ contains everything needed to predict the future; the past adds nothing.""", r"""

\begin{probbox}{the Markov property, expected return and the geometric series}
\textbf{Markov property, as a formula:} $p(s_{t+1}\mid s_t,a_t,s_{t-1},a_{t-1},\dots,s_0)=p(s_{t+1}\mid s_t,a_t)$.

\textbf{Why an expectation.} The dynamics and usually the policy are random, so the return $G$ is a random variable. We maximise its expectation $\E[G]$, the average return over many runs.

\textbf{Why the return is finite.} If every reward satisfies $|r_t|\le r_{\max}$, then $|G|\le r_{\max}\sum_{t\ge0}\gamma^t$. The geometric series: let $S=1+\gamma+\gamma^2+\dots$; then $\gamma S=\gamma+\gamma^2+\dots$, and subtracting, $S-\gamma S=1$, so $S=\dfrac{1}{1-\gamma}$ for $0\le\gamma<1$. Hence $|G|\le r_{\max}/(1-\gamma)$. With $\gamma=0.99$ the sum is 100: the discount makes the agent look roughly 100 steps ahead.
\end{probbox}"""))

# ---------------------------------------------------------------- P12 Bellman
B.append(("part2.tex", r"""A neural network trained to satisfy this equation is a \emph{critic}.""", r"""

\begin{probbox}{deriving the Bellman equation}
\textbf{Step 1: the return is recursive.} $G_t=r_t+\gamma r_{t+1}+\gamma^2r_{t+2}+\dots=r_t+\gamma\,(r_{t+1}+\gamma r_{t+2}+\dots)=r_t+\gamma G_{t+1}$.

\textbf{Step 2: take expectations.} By definition $Q^\pi(s,a)=\E[G_t\mid s_t=s,a_t=a]$. By linearity,
\[
Q^\pi(s,a)=r(s,a)+\gamma\,\E[G_{t+1}\mid s_t=s,a_t=a].
\]
\textbf{Step 3: the law of total expectation}, $\E[X]=\E_Y\big[\E[X\mid Y]\big]$ (average within each case of $Y$, then average over the cases). Condition on the next state $s'$:
\[
\E[G_{t+1}\mid s,a]=\E_{s'\sim p(\cdot\mid s,a)}\big[\E[G_{t+1}\mid s_{t+1}=s']\big]=\E_{s'}\big[V^\pi(s')\big].
\]
The Markov property lets us drop $s,a$ inside the inner expectation: once $s'$ is known, the earlier state adds nothing. Substituting gives equation~\eqref{eq:bellman}.
\end{probbox}"""))

# ---------------------------------------------------------------- P13 log derivative trick
B.append(("part2.tex", r"""An \textbf{actor--critic} method trains a policy (actor) and a value network (critic) together. Dreamer does exactly this, but inside its world model.""", r"""

\begin{probbox}{deriving the policy gradient (the log-derivative trick)}
We want $\nabla_\theta\E_{a\sim\pi_\theta}[f(a)]$, the gradient of an average whose \emph{distribution} depends on $\theta$. Write it as a sum and use $\nabla\pi=\pi\,\nabla\log\pi$ (because $\nabla\log\pi=\nabla\pi/\pi$):
\[
\nabla_\theta\sum_a\pi_\theta(a)f(a)=\sum_a\nabla_\theta\pi_\theta(a)\,f(a)=\sum_a\pi_\theta(a)\,\nabla_\theta\log\pi_\theta(a)\,f(a)=\E_{a\sim\pi_\theta}\big[f(a)\,\nabla_\theta\log\pi_\theta(a)\big].
\]
The right side is an expectation again, so it can be estimated from sampled actions without knowing how $f$ (the environment) works. For whole trajectories, the probability is a product of policy terms and dynamics terms; the log turns it into a sum, and the dynamics terms vanish from the gradient because they do not depend on $\theta$.

\textbf{Why subtracting a baseline is allowed.} For any $b$ that does not depend on the action,
$\E[b\,\nabla\log\pi]=b\sum_a\nabla\pi(a)=b\,\nabla\sum_a\pi(a)=b\,\nabla1=0$. So replacing $f$ by $f-V(s)$, the advantage, does not change the expected gradient, but it makes the estimate much less noisy.
\end{probbox}"""))

# ---------------------------------------------------------------- P14 covariate shift bound
B.append(("part2.tex", r"""instead of the $\epsilon T$ you would expect from independent mistakes.""", r"""

\begin{probbox}{distribution shift, and where $\epsilon T^2$ comes from}
\textbf{Covariate shift} means the distribution of inputs changes between training and test while the correct input--output rule stays the same. Behaviour cloning minimises $\E_{s\sim d_{\text{expert}}}[\ell(s)]$, but the cost we care about is $\E_{s\sim d_{\pi}}[\ell(s)]$, where $d_\pi$ is the distribution of states the learner itself visits.

\textbf{Worst-case argument.} Suppose the learner makes a mistake with probability at most $\epsilon$ in any state the expert visits, and that after its first mistake it is lost and pays cost 1 at every remaining step. The probability that the first mistake happens at step $t$ is at most $\epsilon$ (union bound), and it then pays at most $T-t+1$. So
\[
\E[\text{cost}]\ \le\ \sum_{t=1}^{T}\epsilon\,(T-t+1)=\epsilon\,\frac{T(T+1)}{2}\approx\frac{\epsilon T^2}{2}.
\]
If mistakes did not push the learner off its data, each step would independently cost $\epsilon$ in expectation, $\epsilon T$ in total. DAgger trains on $d_\pi$ itself, which brings the cost back to order $\epsilon T$. Ross and Bagnell also show that the quadratic bound can actually be reached.
\end{probbox}"""))

# ---------------------------------------------------------------- P15 quantisation error
B.append(("part3.tex", r"""A 7-dimensional arm action (3 position, 3 rotation, 1 gripper) is 7 tokens, generated one by one.
\end{examplebox}""", r"""

\begin{probbox}{the rounding error of action bins: the uniform distribution}
If the true value is equally likely to be anywhere inside its bin of width $\Delta$, the rounding error $e$ is \textbf{uniform} on $[-\Delta/2,\Delta/2]$, with density $1/\Delta$ there. Its mean is 0 by symmetry, and its variance is
\[
\mathrm{Var}(e)=\int_{-\Delta/2}^{\Delta/2}e^2\,\frac{1}{\Delta}\,\mathrm de=\frac1\Delta\Big[\frac{e^3}{3}\Big]_{-\Delta/2}^{\Delta/2}=\frac1\Delta\cdot\frac{2}{3}\cdot\frac{\Delta^3}{8}=\frac{\Delta^2}{12}.
\]
So the typical (root-mean-square) error is $\Delta/\sqrt{12}\approx0.29\,\Delta$. For $\Delta=0.0078$~m/s that is about $0.0023$~m/s, and the worst case is $\Delta/2\approx0.0039$~m/s.
\end{probbox}"""))

# ---------------------------------------------------------------- P16 compounding error derivation
B.append(("part4.tex", r"""\label{fig:compound}
\end{figure}""", r"""

\begin{probbox}{why the error grows: a derivation for linear models, and the $1/\sqrt N$ rule}
\textbf{Rollout error.} True system $\x_{h}=A^h\x_0$; model $\hat\x_h=\hat A^h\x_0$ with $\hat A=A+E$ and a small error $E$. Expanding the product and keeping terms with one $E$,
\[
\hat A^h=A^h+\sum_{j=0}^{h-1}A^{j}\,E\,A^{h-1-j}+(\text{terms with }E^2,E^3,\dots).
\]
For a rotation, $\lVert A\rVert=1$, so each of the $h$ terms has size at most $\lVert E\rVert$: the error is at most about $h\,\lVert E\rVert\,\lVert\x_0\rVert$, growing linearly with the horizon. In Figure~\ref{fig:compound} it grows more slowly than this bound because the rotated error terms partly cancel. For an unstable system ($\lVert A\rVert>1$) the same sum grows exponentially, which is why long imagined rollouts of fast, unstable dynamics such as a drone's are hard.

\textbf{Where $E$ comes from.} Least squares on $N$ noisy transitions estimates each entry of $A$ with a standard error proportional to $\sigma_{\text{noise}}/\sqrt N$, the same $1/\sqrt N$ law as for a sample mean ($\mathrm{Var}(\bar X)=\sigma^2/N$ by the variance rules in Part I). Four times the data halves the model error and so roughly halves the rollout error.
\end{probbox}"""))

# ---------------------------------------------------------------- P17 ELBO
B.append(("part4.tex", r"""At planning time only the prior is used, so the model can roll forward without any images at all.""", r"""

\begin{probbox}{Bayes' rule, prior and posterior, and the derivation of the ELBO}
\textbf{Bayes' rule.} From $p(z,o)=p(z)p(o\mid z)=p(o)p(z\mid o)$:
\[
p(z\mid o)=\frac{p(o\mid z)\,p(z)}{p(o)},\qquad p(o)=\int p(o\mid z)\,p(z)\,\mathrm dz .
\]
The \textbf{prior} $p(z)$ is the belief before seeing the image; the \textbf{posterior} $p(z\mid o)$ is the belief after. The integral for $p(o)$ runs over all possible latents, so the posterior cannot be computed exactly. A VAE or RSSM therefore learns an approximation $q(z\mid o)$, the encoder.

\textbf{Derivation of the ELBO} (evidence lower bound). Multiply and divide by $q$ inside the integral, then use Jensen ($\log\E\ge\E\log$, because $\log$ is concave):
\[
\log p(o)=\log\E_{z\sim q}\Big[\frac{p(o\mid z)\,p(z)}{q(z\mid o)}\Big]\ \ge\ \E_q\big[\log p(o\mid z)\big]+\E_q\Big[\log\frac{p(z)}{q(z\mid o)}\Big]=\underbrace{\E_q[\log p(o\mid z)]}_{\text{reconstruction}}-\underbrace{\mathrm{KL}\big(q(z\mid o)\,\|\,p(z)\big)}_{\text{stay close to the prior}} .
\]
Maximising this bound makes the data likely while keeping the encoder's guesses close to the prior. Its negative is the VAE loss, and the RSSM loss~\eqref{eq:rssmloss} is the same bound written for every time step, with the learned prior $p(\hat\z_t\mid\h_t)$ in place of a fixed $\mathcal N(0,1)$. One can show that the gap in the inequality is exactly $\mathrm{KL}\big(q(z\mid o)\,\|\,p(z\mid o)\big)$: the bound is tight when the encoder matches the true posterior.
\end{probbox}"""))

# ---------------------------------------------------------------- P18 CEM MLE, MPPI
B.append(("part4.tex", r"""\textbf{MPPI} is a close relative that weights all samples by $e^{-\text{cost}/\lambda}$ instead of keeping a hard set of elites; TD-MPC2 uses it.""", r"""

\begin{probbox}{CEM refits a Gaussian by maximum likelihood; MPPI is a softmax}
\textbf{Derivation.} Fit $\mathcal N(\mu,\sigma^2)$ to the $K$ elites $x_1,\dots,x_K$ by maximum likelihood (first box of Part I). The log-likelihood is
\[
\ell(\mu,\sigma^2)=-\frac K2\log(2\pi\sigma^2)-\sum_{i=1}^K\frac{(x_i-\mu)^2}{2\sigma^2}.
\]
Setting $\partial\ell/\partial\mu=\sum_i(x_i-\mu)/\sigma^2=0$ gives $\mu=\frac1K\sum_ix_i$, the sample mean. Setting $\partial\ell/\partial\sigma^2=-\frac{K}{2\sigma^2}+\frac{\sum_i(x_i-\mu)^2}{2\sigma^4}=0$ gives $\sigma^2=\frac1K\sum_i(x_i-\mu)^2$, the sample variance. So step 4 of CEM is maximum likelihood on the elites. The name: this is the same as minimising the cross-entropy between the elite distribution and the sampling distribution. (In the toy of Figure~\ref{fig:cem}, $0.02$ is added to $\sigma$ so that the search never collapses completely.)

\textbf{MPPI.} The weights $w_i=e^{-c_i/\lambda}/\sum_je^{-c_j/\lambda}$ are a softmax of the negative costs at temperature $\lambda$ (see the softmax box). As $\lambda\to0$ only the best sample counts; as $\lambda\to\infty$ all samples count equally. The new mean is $\sum_iw_ix_i$.
\end{probbox}"""))

# ---------------------------------------------------------------- P19 success rates statistics
B.append(("part5.tex", r"""which fits a master's thesis.
\end{opinionbox}""", r"""

\begin{probbox}{how many trials you need to compare two success rates}
Each trial succeeds or fails, a Bernoulli variable with success probability $p$ and variance $p(1-p)$. The measured success rate $\hat p$ over $n$ independent trials is a mean of $n$ such variables, so $\E[\hat p]=p$ and $\mathrm{Var}(\hat p)=p(1-p)/n$. Its \textbf{standard error} is $\sqrt{p(1-p)/n}$. By the central limit theorem, $\hat p$ is roughly Gaussian for large $n$, so about 95\% of the time it lies within $1.96$ standard errors of $p$.

\textbf{Difference of two variants} measured on separate trials: variances add, so
\[
\mathrm{SE}(\hat p_2-\hat p_1)=\sqrt{\frac{p_1(1-p_1)}{n_1}+\frac{p_2(1-p_2)}{n_2}} .
\]
\textbf{Example.} 65\% versus 77\%, as in the Planning Limits paper, with $n=100$ trials each: $\mathrm{SE}=\sqrt{(0.2275+0.1771)/100}=0.064$, and the difference $0.12$ is $1.89$ standard errors, just short of the usual $1.96$. For it to pass you need $n\ge0.4046\,(1.96/0.12)^2\approx108$ trials per variant. With 20 trials per variant (common in real-robot papers) the standard error is about $0.14$, larger than the effect. Plan your drone experiments with this in mind: run many trials in simulation, and use the real flights to show that the result transfers.
\end{probbox}"""))

files = {}
for f, anchor, text in B:
    if f not in files:
        files[f] = open(f).read()
    s = files[f]
    n = s.count(anchor)
    if n != 1:
        sys.exit(f"anchor found {n} times in {f}: {anchor[:70]!r}")
    if text.strip() in s:
        sys.exit(f"box already present in {f}: {text.strip()[:60]!r}")
    files[f] = s.replace(anchor, anchor + text)
for f, s in files.items():
    open(f, "w").write(s)
print("inserted", len(B), "boxes into", sorted(files))
